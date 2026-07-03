#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <string>

/**
 * @brief Узел захвата видео с IP-камеры через RTSP.
 * Публикует сжатые кадры H.265 в топик /camera_image.
 */
class VideoCamera : public rclcpp::Node
{
public:
    VideoCamera() : Node("video_camera_node"),
        pipeline_(nullptr),
        appsink_(nullptr),
        frame_count_(0),
        last_log_time_(this->now())
    {
        RCLCPP_INFO(this->get_logger(), "=== VideoCamera Node Started ===");

        // --- 1. Параметры ---
        this->declare_parameter<std::string>("rtsp_url", "rtsp://192.168.144.25:8554/video1");
        std::string rtsp_url = this->get_parameter("rtsp_url").as_string();
        RCLCPP_INFO(this->get_logger(), "RTSP URL: %s", rtsp_url.c_str());

        // --- 2. Издатель ---
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort();
        qos.durability_volatile();
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera_image", qos);

        // --- 3. Инициализация GStreamer ---
        gst_init(nullptr, nullptr);

        // --- 4. Пайплайн ---
        std::string pipeline_str =
            "rtspsrc location=" + rtsp_url + " latency=0 ! "
                                             "rtph265depay ! "
                                             "h265parse ! "
                                             "capsfilter caps=video/x-h265,stream-format=byte-stream ! "
                                             "appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false";

        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        if (!pipeline_) {
            RCLCPP_ERROR(this->get_logger(), "GStreamer pipeline creation failed: %s",
                         error ? error->message : "unknown");
            if (error) g_error_free(error);
            return;
        }

        // --- 5. Получение appsink ---
        GstElement* sink_elem = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        if (!sink_elem) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get 'sink' element");
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            return;
        }
        appsink_ = GST_APP_SINK(sink_elem);
        gst_object_unref(sink_elem);

        // --- 6. Настройка appsink ---
        gst_app_sink_set_emit_signals(appsink_, true);
        gst_app_sink_set_max_buffers(appsink_, 1);
        gst_app_sink_set_drop(appsink_, true);

        // --- 7. Колбэк ---
        g_signal_connect(appsink_, "new-sample",
                         G_CALLBACK(VideoCamera::on_new_sample_static), this);

        // --- 8. Запуск ---
        GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set pipeline to PLAYING");
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "VideoCamera Node ready!");
    }

    ~VideoCamera()
    {
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
        RCLCPP_INFO(this->get_logger(), "VideoCamera Node shutdown");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    GstElement* pipeline_;
    GstAppSink* appsink_;
    unsigned int frame_count_;
    rclcpp::Time last_log_time_;

    static GstFlowReturn on_new_sample_static(GstAppSink* sink, gpointer user_data)
    {
        VideoCamera* node = static_cast<VideoCamera*>(user_data);
        return node->process_gst_sample(sink);
    }

    GstFlowReturn process_gst_sample(GstAppSink* sink)
    {
        GstSample* sample = gst_app_sink_pull_sample(sink);
        if (!sample) {
            RCLCPP_WARN(this->get_logger(), "Failed to pull sample");
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

        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame";
        msg->encoding = "h265";
        msg->data.assign(map.data, map.data + map.size);

        publisher_->publish(*msg);

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        frame_count_++;
        auto now = this->now();
        if ((now - last_log_time_).seconds() >= 1.0) {
            RCLCPP_INFO(this->get_logger(), "[VideoCamera] FPS: %d, Frame size: %.2f KB",
                        frame_count_, map.size / 1024.0);
            frame_count_ = 0;
            last_log_time_ = now;
        }

        return GST_FLOW_OK;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoCamera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}