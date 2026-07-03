#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <array>
#include <string>
#include <sstream>
#include <iomanip>

namespace io = boost::asio;
using io::ip::udp;

class UdpClientNode : public rclcpp::Node
{
public:
    UdpClientNode() : Node("udp_client_node"),
        socket_(io_context_),
        pipeline_(nullptr),
        appsrc_(nullptr),
        appsink_(nullptr),
        frame_count_(0),
        last_log_time_(this->now()),
        current_pitch_(0.0f),
        current_yaw_(0.0f),
        current_mode_(-1),      // -1 означает "неизвестно"
        current_zoom_(0.0f),
        has_mode_zoom_(false)
    {
        RCLCPP_INFO(this->get_logger(), "=== UDP H.265 CLIENT ===");

        this->declare_parameter<int>("udp_port", 12346);
        int udp_port = this->get_parameter("udp_port").as_int();
        if (udp_port < 1 || udp_port > 65535) {
            RCLCPP_ERROR(this->get_logger(), "Invalid UDP port %d, using default 12346", udp_port);
            udp_port = 12346;
        }
        RCLCPP_INFO(this->get_logger(), "Listening on port: %d", udp_port);

        try {
            socket_.open(udp::v4());
            socket_.bind(udp::endpoint(udp::v4(), udp_port));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind UDP socket: %s", e.what());
            throw;
        }

        cv::namedWindow("UDP Video", cv::WINDOW_NORMAL);
        cv::resizeWindow("UDP Video", 640, 480);

        gst_init(nullptr, nullptr);

        std::string pipeline_str =
            "appsrc name=src format=time is-live=true block=false ! "
            "h265parse ! avdec_h265 ! videoconvert ! "
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

        GstElement* src_elem = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
        if (!src_elem) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get 'src' element");
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            throw std::runtime_error("Missing 'src' element");
        }
        appsrc_ = GST_APP_SRC(src_elem);
        gst_object_unref(src_elem);

        GstCaps* caps = gst_caps_new_simple("video/x-h265",
                                            "stream-format", G_TYPE_STRING, "byte-stream",
                                            nullptr);
        gst_app_src_set_caps(appsrc_, caps);
        gst_caps_unref(caps);
        gst_app_src_set_size(appsrc_, -1);
        gst_app_src_set_stream_type(appsrc_, GST_APP_STREAM_TYPE_STREAM);

        GstElement* sink_elem = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        if (!sink_elem) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get 'sink' element");
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            throw std::runtime_error("Missing 'sink' element");
        }
        appsink_ = GST_APP_SINK(sink_elem);
        gst_object_unref(sink_elem);

        gst_app_sink_set_emit_signals(appsink_, true);
        gst_app_sink_set_max_buffers(appsink_, 1);
        gst_app_sink_set_drop(appsink_, true);

        g_signal_connect(appsink_, "new-sample",
                         G_CALLBACK(UdpClientNode::on_new_sample_static), this);

        GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set pipeline to PLAYING");
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            throw std::runtime_error("GStreamer start failed");
        }

        state_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/tcp_received/angles", 10,
            std::bind(&UdpClientNode::state_callback, this, std::placeholders::_1));

        start_receive();

        io_thread_ = std::thread([this]() { io_context_.run(); });

        RCLCPP_INFO(this->get_logger(), "UDP Client ready! Subscribed to /gimbal/angles");
    }

    ~UdpClientNode()
    {
        io_context_.stop();
        if (io_thread_.joinable()) io_thread_.join();

        if (socket_.is_open()) {
            boost::system::error_code ec;
            socket_.close(ec);
        }

        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }

        cv::destroyAllWindows();
        RCLCPP_INFO(this->get_logger(), "UDP Client shutdown");
    }

private:
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

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub_;
    float current_pitch_;
    float current_yaw_;
    int current_mode_;          // -1 = неизвестно
    float current_zoom_;
    bool has_mode_zoom_;        // true если получены mode и zoom

    void start_receive()
    {
        socket_.async_receive_from(
            io::buffer(recv_buffer_),
            remote_endpoint_,
            [this](const boost::system::error_code& error, size_t bytes_transferred) {
                if (!error && bytes_transferred > 0) {
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
                start_receive();
            }
            );
    }

    // --- Исправленный колбэк для состояния камеры ---
    void state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 2) {
            current_pitch_ = msg->data[0];
            current_yaw_ = msg->data[1];
        }
        if (msg->data.size() >= 4) {
            current_mode_ = static_cast<int>(msg->data[2]);
            current_zoom_ = msg->data[3];
            has_mode_zoom_ = true;
        } else {
            has_mode_zoom_ = false;
        }
        RCLCPP_DEBUG(this->get_logger(), "State update: pitch=%.1f, yaw=%.1f, mode=%d, zoom=%.2f, has_mode_zoom=%d",
                     current_pitch_, current_yaw_, current_mode_, current_zoom_, has_mode_zoom_);
    }

    static GstFlowReturn on_new_sample_static(GstAppSink* sink, gpointer user_data)
    {
        UdpClientNode* node = static_cast<UdpClientNode*>(user_data);
        return node->process_gst_sample(sink);
    }

    GstFlowReturn process_gst_sample(GstAppSink* sink)
    {
        GstSample* sample = gst_app_sink_pull_sample(sink);
        if (!sample) return GST_FLOW_ERROR;

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

        GstCaps* caps = gst_sample_get_caps(sample);
        GstStructure* structure = gst_caps_get_structure(caps, 0);
        int width, height;
        gst_structure_get_int(structure, "width", &width);
        gst_structure_get_int(structure, "height", &height);

        cv::Mat image(height, width, CV_8UC3, map.data);

        if (!image.empty()) {
            // --- Формирование строк состояния ---
            std::string mode_str;
            if (has_mode_zoom_) {
                switch (current_mode_) {
                case 0: mode_str = "RGB"; break;
                case 1: mode_str = "Thermal"; break;
                case 2: mode_str = "Wide"; break;
                default: mode_str = "Unknown";
                }
            } else {
                mode_str = "N/A";
            }

            std::string zoom_str;
            if (has_mode_zoom_) {
                std::ostringstream ss;
                ss << std::fixed << std::setprecision(2) << current_zoom_;
                zoom_str = ss.str();
            } else {
                zoom_str = "N/A";
            }

            // Pitch
            std::ostringstream pitch_ss;
            pitch_ss << std::fixed << std::setprecision(1) << current_pitch_ << "deg";
            std::string pitch_str = pitch_ss.str();

            std::ostringstream yaw_ss;
            yaw_ss << std::fixed << std::setprecision(1) << current_yaw_ << "deg";
            std::string yaw_str = yaw_ss.str();

            // --- Отрисовка ---
            int start_x = 15;
            int start_y = 45;
            int line_height = 45;
            double font_size = 1.0;
            int thickness = 2;

            // --- Крестовидный прицел ---
            int center_x = width / 2;
            int center_y = height / 2;
            int cross_size = 30;        // длина каждой линии от центра
            int square_size = 10;       // половина стороны квадрата
            cv::Scalar cross_color(0, 255, 0); // ярко-зелёный

            // Квадрат в центре
            cv::rectangle(image,
                          cv::Point(center_x - square_size, center_y - square_size),
                          cv::Point(center_x + square_size, center_y + square_size),
                          cross_color, 2);

            // Вертикальные линии (вверх и вниз)
            cv::line(image, cv::Point(center_x, center_y - cross_size),
                     cv::Point(center_x, center_y - square_size), cross_color, 2);
            cv::line(image, cv::Point(center_x, center_y + square_size),
                     cv::Point(center_x, center_y + cross_size), cross_color, 2);

            // Горизонтальные линии (влево и вправо)
            cv::line(image, cv::Point(center_x - cross_size, center_y),
                     cv::Point(center_x - square_size, center_y), cross_color, 2);
            cv::line(image, cv::Point(center_x + square_size, center_y),
                     cv::Point(center_x + cross_size, center_y), cross_color, 2);

            // Фон
            cv::rectangle(image,
                          cv::Point(start_x - 10, start_y - 35),
                          cv::Point(start_x + 290, start_y + 4 * line_height + 25),
                          cv::Scalar(0, 0, 0),
                          cv::FILLED);
            cv::rectangle(image,
                          cv::Point(start_x - 10, start_y - 35),
                          cv::Point(start_x + 290, start_y + 4 * line_height + 25),
                          cv::Scalar(180, 180, 180),
                          2);

            // Заголовок
            cv::putText(image, "CAMERA STATE",
                        cv::Point(start_x, start_y),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.9,
                        cv::Scalar(255, 255, 255),
                        2);

            // Pitch
            cv::putText(image, "Pitch:  " + pitch_str,
                        cv::Point(start_x, start_y + line_height),
                        cv::FONT_HERSHEY_SIMPLEX,
                        font_size,
                        cv::Scalar(255, 200, 0),
                        thickness);

            // Yaw
            cv::putText(image, "Yaw:    " + yaw_str,
                        cv::Point(start_x, start_y + 2 * line_height),
                        cv::FONT_HERSHEY_SIMPLEX,
                        font_size,
                        cv::Scalar(0, 255, 0),
                        thickness);

            // Mode
            cv::putText(image, "Mode:   " + mode_str,
                        cv::Point(start_x, start_y + 3 * line_height),
                        cv::FONT_HERSHEY_SIMPLEX,
                        font_size,
                        cv::Scalar(0, 200, 255),
                        thickness);

            // Zoom
            cv::putText(image, "Zoom:   " + zoom_str,
                        cv::Point(start_x, start_y + 4 * line_height),
                        cv::FONT_HERSHEY_SIMPLEX,
                        font_size,
                        cv::Scalar(255, 0, 255),
                        thickness);
        }

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