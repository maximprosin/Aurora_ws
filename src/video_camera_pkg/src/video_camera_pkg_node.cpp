#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

// ★ Подключаем GStreamer ★
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

class VideoCamera : public rclcpp::Node {
public:
    VideoCamera() :
        Node("video_camera_node"),
        // ★ Инициализация публикатора ★
        publisher_(nullptr),
        // ★ Инициализация GStreamer ★
        gst_pipeline_(nullptr),
        gst_appsink_(nullptr),
        // ★ Счетчики ★
        pub_frame_count_(0),
        sub_frame_count_(0),
        pub_last_log_time_(this->now()),
        sub_last_log_time_(this->now()),
        // ★ Настройки ★
        enable_publisher_(true),   // Включить публикатор
        enable_subscriber_(true),  // Включить подписчик
        show_video_window_(true),  // Показывать окно с видео
        rtsp_url_("rtsp://192.168.144.25:8554/video2")
    {
        RCLCPP_INFO(this->get_logger(), "=== VideoCamera Node Started ===");
        RCLCPP_INFO(this->get_logger(), "RTSP URL: %s", rtsp_url_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publisher: %s", enable_publisher_ ? "ON" : "OFF");
        RCLCPP_INFO(this->get_logger(), "Subscriber: %s", enable_subscriber_ ? "ON" : "OFF");
        RCLCPP_INFO(this->get_logger(), "Video Window: %s", show_video_window_ ? "ON" : "OFF");

        // ★ 1. Создаем публикатор (если включен) ★
        if (enable_publisher_) {
            setup_publisher();
        }

        // ★ 2. Создаем подписчик (если включен) ★
        if (enable_subscriber_) {
            setup_subscriber();
        }

        // ★ 3. Запускаем GStreamer захват (если включен публикатор) ★
        if (enable_publisher_) {
            setup_gstreamer();
        }

        RCLCPP_INFO(this->get_logger(), "VideoCamera Node ready!");
    }

    ~VideoCamera() {
        // ★ Очистка GStreamer ★
        if (gst_pipeline_) {
            gst_element_set_state(gst_pipeline_, GST_STATE_NULL);
            gst_object_unref(gst_pipeline_);
        }
        // ★ Закрываем окно OpenCV ★
        if (show_video_window_) {
            cv::destroyWindow("Camera Stream");
        }
        RCLCPP_INFO(this->get_logger(), "VideoCamera Node shutdown");
    }

private:
    // ★ ROS2 компоненты ★
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    // ★ GStreamer компоненты ★
    GstElement* gst_pipeline_;
    GstAppSink* gst_appsink_;

    // ★ Счетчики и статистика ★
    unsigned int pub_frame_count_;
    unsigned int sub_frame_count_;
    rclcpp::Time pub_last_log_time_;
    rclcpp::Time sub_last_log_time_;
    rclcpp::Time last_frame_time_;

    // ★ Настройки ★
    bool enable_publisher_;
    bool enable_subscriber_;
    bool show_video_window_;
    std::string rtsp_url_;
    bool format_logged_ = false;

    // ============================================================
    // ★ 1. НАСТРОЙКА ПУБЛИКАТОРА ★
    // ============================================================
    void setup_publisher() {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort();
        qos.durability_volatile();

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera_image", qos);

        RCLCPP_INFO(this->get_logger(), "Publisher created on topic: /camera_image");
    }

    // ============================================================
    // ★ 2. НАСТРОЙКА ПОДПИСЧИКА ★
    // ============================================================
    void setup_subscriber() {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort();
        qos.durability_volatile();

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_image",
            qos,
            std::bind(&VideoCamera::subscriber_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscriber created on topic: /camera_image");

        // ★ Создаем окно для отображения ★
        if (show_video_window_) {
            cv::namedWindow("Camera Stream", cv::WINDOW_NORMAL);
            cv::resizeWindow("Camera Stream", 640, 480);
        }

        last_frame_time_ = this->now();
    }

    // ============================================================
    // ★ 3. НАСТРОЙКА GSTREAMER ★
    // ============================================================
    void setup_gstreamer() {
        gst_init(nullptr, nullptr);

        // ★ Собираем pipeline ★
        std::string pipeline_str =
            "rtspsrc location=" + rtsp_url_ + " latency=0 ! "
                                              "rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! "
                                              "video/x-raw,format=BGR ! "
                                              "appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false";

        RCLCPP_INFO(this->get_logger(), "GStreamer pipeline: %s", pipeline_str.c_str());

        GError* error = nullptr;
        gst_pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);

        if (!gst_pipeline_) {
            RCLCPP_ERROR(this->get_logger(),
                         "GStreamer error: %s",
                         error ? error->message : "unknown");
            if (error) g_error_free(error);
            return;
        }

        // ★ Получаем appsink ★
        GstElement* sink = gst_bin_get_by_name(GST_BIN(gst_pipeline_), "sink");
        if (!sink) {
            RCLCPP_ERROR(this->get_logger(), "Can't get appsink!");
            return;
        }
        gst_appsink_ = GST_APP_SINK(sink);
        gst_object_unref(sink);

        // ★ Настраиваем appsink ★
        gst_app_sink_set_emit_signals(gst_appsink_, true);
        gst_app_sink_set_max_buffers(gst_appsink_, 1);
        gst_app_sink_set_drop(gst_appsink_, true);

        // ★ Подключаем колбэк ★
        g_signal_connect(gst_appsink_, "new-sample",
                         G_CALLBACK(on_new_sample), this);

        // ★ Запускаем ★
        gst_element_set_state(gst_pipeline_, GST_STATE_PLAYING);

        RCLCPP_INFO(this->get_logger(), "GStreamer pipeline started!");
    }

    // ============================================================
    // ★ 4. КОЛБЭК GSTREAMER (захват кадров) ★
    // ============================================================
    static GstFlowReturn on_new_sample(GstAppSink* sink, gpointer user_data) {
        VideoCamera* node = static_cast<VideoCamera*>(user_data);
        return node->process_gst_sample(sink);
    }

    GstFlowReturn process_gst_sample(GstAppSink* sink) {
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

        // ★ Получаем информацию о кадре ★
        GstCaps* caps = gst_sample_get_caps(sample);
        GstStructure* structure = gst_caps_get_structure(caps, 0);

        int width = 0, height = 0;
        gst_structure_get_int(structure, "width", &width);
        gst_structure_get_int(structure, "height", &height);

        // ★ Определяем формат ★
        const char* format = gst_structure_get_string(structure, "format");
        int bpp = 3;

        if (format) {
            if (strcmp(format, "BGR") == 0 || strcmp(format, "RGB") == 0) {
                bpp = 3;
            } else if (strcmp(format, "BGRA") == 0 || strcmp(format, "RGBA") == 0 ||
                       strcmp(format, "BGRx") == 0 || strcmp(format, "RGBx") == 0) {
                bpp = 4;
            } else {
                size_t expected_bgr = width * height * 3;
                size_t expected_bgra = width * height * 4;
                bpp = (map.size == expected_bgr) ? 3 :
                          (map.size == expected_bgra) ? 4 : 3;
            }

            if (!format_logged_) {
                RCLCPP_INFO(this->get_logger(),
                            "GStreamer format: %s, bpp=%d, %dx%d",
                            format, bpp, width, height);
                format_logged_ = true;
            }
        }

        // ★ Создаем ROS сообщение ★
        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame";
        msg->height = height;
        msg->width = width;
        msg->encoding = (bpp == 3) ? "bgr8" : "bgra8";
        msg->is_bigendian = false;
        msg->step = width * bpp;
        msg->data.resize(map.size);
        memcpy(msg->data.data(), map.data, map.size);

        // ★ Публикуем ★
        if (enable_publisher_) {
            publisher_->publish(*msg);
        }

        // ★ Считаем FPS публикатора ★
        pub_frame_count_++;
        if ((this->now() - pub_last_log_time_).seconds() >= 1.0) {
            RCLCPP_INFO(this->get_logger(), "[PUB] FPS: %d", pub_frame_count_);
            pub_frame_count_ = 0;
            pub_last_log_time_ = this->now();
        }

        // ★ Если подписчик выключен, показываем видео прямо здесь ★
        if (!enable_subscriber_ && show_video_window_) {
            cv::Mat frame(height, width, CV_8UC3, map.data);
            if (!frame.empty()) {
                cv::imshow("Camera Stream (Direct)", frame);
                cv::waitKey(1);
            }
        }

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        return GST_FLOW_OK;
    }

    // ============================================================
    // ★ 5. КОЛБЭК ПОДПИСЧИКА ★
    // ============================================================
    void subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto now = this->now();
        auto delay = (now - last_frame_time_).seconds() * 1000;
        last_frame_time_ = now;

        sub_frame_count_++;

        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            if (!frame.empty() && show_video_window_) {
                // ★ Добавляем информацию на кадр ★
                std::string info_text = "Frames: " + std::to_string(sub_frame_count_) +
                                        " Delay: " + std::to_string((int)delay) + "ms";
                cv::putText(frame, info_text, cv::Point(10, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                cv::imshow("Camera Stream", frame);
                cv::waitKey(1);
            }

            // ★ Логируем FPS подписчика ★
            if ((now - sub_last_log_time_).seconds() >= 1.0) {
                RCLCPP_INFO(this->get_logger(),
                            "[SUB] FPS: %d, delay: %.0f ms",
                            sub_frame_count_, delay);
                sub_frame_count_ = 0;
                sub_last_log_time_ = now;
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Subscriber error: %s", e.what());
        }
    }
};

// ============================================================
// ★ MAIN ★
// ============================================================
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<VideoCamera>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}