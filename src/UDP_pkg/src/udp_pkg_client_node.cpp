#include <rclcpp/rclcpp.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <array>

namespace io = boost::asio;
using io::ip::udp;

/**
 * @brief UDP-клиент для приёма H.265 видео и отображения через OpenCV.
 */
class UdpClientNode : public rclcpp::Node
{
public:
    UdpClientNode() : Node("udp_client_node"),
        socket_(io_context_),
        pipeline_(nullptr),
        appsrc_(nullptr),
        appsink_(nullptr),
        frame_count_(0),
        last_log_time_(this->now())
    {
        RCLCPP_INFO(this->get_logger(), "=== UDP H.265 CLIENT ===");

        // --- 1. Параметры ---
        this->declare_parameter<int>("udp_port", 12346);
        int udp_port = this->get_parameter("udp_port").as_int();
        if (udp_port < 1 || udp_port > 65535) {
            RCLCPP_ERROR(this->get_logger(), "Invalid UDP port %d, using default 12346", udp_port);
            udp_port = 12346;
        }
        RCLCPP_INFO(this->get_logger(), "Listening on port: %d", udp_port);

        // --- 2. Открытие UDP-сокета ---
        try {
            socket_.open(udp::v4());
            socket_.bind(udp::endpoint(udp::v4(), udp_port));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind UDP socket: %s", e.what());
            throw;
        }

        // --- 3. Создание окна OpenCV ---
        cv::namedWindow("UDP Video", cv::WINDOW_NORMAL);
        cv::resizeWindow("UDP Video", 640, 480);

        // --- 4. Инициализация GStreamer ---
        gst_init(nullptr, nullptr);

        // --- 5. Сборка пайплайна декодирования ---
        std::string pipeline_str =
            "appsrc name=src format=time is-live=true block=false ! "
            "h265parse ! "
            "avdec_h265 ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false";

        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        if (!pipeline_) {
            RCLCPP_ERROR(this->get_logger(), "GStreamer pipeline creation failed: %s",
                         error ? error->message : "unknown");
            if (error) g_error_free(error);
            throw std::runtime_error("GStreamer pipeline creation failed");
        }

        // --- 6. Получение appsrc ---
        GstElement* src_elem = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
        if (!src_elem) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get 'src' element");
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            throw std::runtime_error("Missing 'src' element");
        }
        appsrc_ = GST_APP_SRC(src_elem);
        gst_object_unref(src_elem);

        // --- 7. Настройка appsrc ---
        GstCaps* caps = gst_caps_new_simple("video/x-h265",
                                            "stream-format", G_TYPE_STRING, "byte-stream",
                                            nullptr);
        gst_app_src_set_caps(appsrc_, caps);
        gst_caps_unref(caps);
        gst_app_src_set_size(appsrc_, -1);
        gst_app_src_set_stream_type(appsrc_, GST_APP_STREAM_TYPE_STREAM);

        // --- 8. Получение appsink ---
        GstElement* sink_elem = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        if (!sink_elem) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get 'sink' element");
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            throw std::runtime_error("Missing 'sink' element");
        }
        appsink_ = GST_APP_SINK(sink_elem);
        gst_object_unref(sink_elem);

        // --- 9. Настройка appsink ---
        gst_app_sink_set_emit_signals(appsink_, true);
        gst_app_sink_set_max_buffers(appsink_, 1);
        gst_app_sink_set_drop(appsink_, true);

        // --- 10. Подключение колбэка на новый кадр ---
        g_signal_connect(appsink_, "new-sample",
                         G_CALLBACK(UdpClientNode::on_new_sample_static), this);

        // --- 11. Запуск пайплайна ---
        GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set pipeline to PLAYING");
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            throw std::runtime_error("GStreamer start failed");
        }

        // --- 12. Запуск приёма UDP ---
        start_receive();

        // --- 13. Запуск потока io_context ---
        io_thread_ = std::thread([this]() { io_context_.run(); });

        RCLCPP_INFO(this->get_logger(), "UDP Client ready!");
    }

    ~UdpClientNode()
    {
        // Остановка io_context
        io_context_.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }

        // Закрытие сокета
        if (socket_.is_open()) {
            boost::system::error_code ec;
            socket_.close(ec);
        }

        // Остановка GStreamer пайплайна
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }

        // Закрытие окон OpenCV
        cv::destroyAllWindows();

        RCLCPP_INFO(this->get_logger(), "UDP Client shutdown");
    }

private:
    // --- Члены класса ---
    io::io_context io_context_;
    udp::socket socket_;
    udp::endpoint remote_endpoint_;
    std::thread io_thread_;

    GstElement* pipeline_;
    GstAppSrc* appsrc_;
    GstAppSink* appsink_;

    std::array<char, 65536> recv_buffer_;
    unsigned int frame_count_;
    rclcpp::Time last_log_time_;

    // --- Запуск асинхронного приёма ---
    void start_receive()
    {
        socket_.async_receive_from(
            io::buffer(recv_buffer_),
            remote_endpoint_,
            [this](const boost::system::error_code& error, size_t bytes_transferred) {
                if (!error && bytes_transferred > 0) {
                    // Передаём данные в appsrc
                    GstBuffer* buffer = gst_buffer_new_and_alloc(bytes_transferred);
                    GstMapInfo map;
                    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
                    memcpy(map.data, recv_buffer_.data(), bytes_transferred);
                    gst_buffer_unmap(buffer, &map);

                    GstFlowReturn ret = gst_app_src_push_buffer(appsrc_, buffer);
                    if (ret != GST_FLOW_OK) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1.0,
                                             "Failed to push buffer to appsrc");
                    }

                    // Статистика
                    frame_count_++;
                    auto now = this->now();
                    if ((now - last_log_time_).seconds() >= 1.0) {
                        RCLCPP_INFO(this->get_logger(), "[UDP Client] FPS: %d, Packet size: %.2f KB",
                                    frame_count_, bytes_transferred / 1024.0);
                        frame_count_ = 0;
                        last_log_time_ = now;
                    }
                } else if (error) {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1.0,
                                          "UDP receive error: %s", error.message().c_str());
                }
                // Продолжаем приём
                start_receive();
            }
            );
    }

    // --- Статический колбэк для GStreamer (новый кадр) ---
    static GstFlowReturn on_new_sample_static(GstAppSink* sink, gpointer user_data)
    {
        UdpClientNode* node = static_cast<UdpClientNode*>(user_data);
        return node->process_gst_sample(sink);
    }

    // --- Обработка декодированного кадра ---
    GstFlowReturn process_gst_sample(GstAppSink* sink)
    {
        GstSample* sample = gst_app_sink_pull_sample(sink);
        if (!sample) {
            return GST_FLOW_ERROR;
        }

        GstBuffer* buffer = gst_sample_get_buffer(sample);
        if (!buffer) {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

        // Конвертация в OpenCV Mat и отображение
        cv::Mat image(cv::Size(640, 480), CV_8UC3, map.data);
        cv::imshow("UDP Video", image);
        cv::waitKey(1);

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        return GST_FLOW_OK;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<UdpClientNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("UdpClientNode"), "Fatal error: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}