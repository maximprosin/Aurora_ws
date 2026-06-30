#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

class VideoCamera : public rclcpp::Node {
public:
    VideoCamera() :
        Node("video_camera_node"),
        publisher_(nullptr),
        gst_pipeline_(nullptr),
        gst_appsink_(nullptr),
        frame_count_(0),
        last_log_time_(this->now())
    {
        RCLCPP_INFO(this->get_logger(), "=== VideoCamera Node Started ===");

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort();
        qos.durability_volatile();

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera_image", qos);

        gst_init(nullptr, nullptr);

        // ★ ИСПРАВЛЕНИЕ: добавляем capsfilter для byte-stream ★
        std::string pipeline_str =
            "rtspsrc location=rtsp://192.168.144.25:8554/video2 latency=0 ! "
            "rtph265depay ! h265parse ! "
            "capsfilter caps=video/x-h265,stream-format=byte-stream ! "
            "appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false";

        RCLCPP_INFO(this->get_logger(), "GStreamer pipeline: %s", pipeline_str.c_str());

        GError* error = nullptr;
        gst_pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);

        if (!gst_pipeline_) {
            RCLCPP_ERROR(this->get_logger(), "GStreamer error: %s",
                         error ? error->message : "unknown");
            if (error) g_error_free(error);
            return;
        }

        GstElement* sink = gst_bin_get_by_name(GST_BIN(gst_pipeline_), "sink");
        gst_appsink_ = GST_APP_SINK(sink);
        gst_object_unref(sink);

        gst_app_sink_set_emit_signals(gst_appsink_, true);
        gst_app_sink_set_max_buffers(gst_appsink_, 1);
        gst_app_sink_set_drop(gst_appsink_, true);

        g_signal_connect(gst_appsink_, "new-sample",
                         G_CALLBACK(on_new_sample), this);

        gst_element_set_state(gst_pipeline_, GST_STATE_PLAYING);

        RCLCPP_INFO(this->get_logger(), "VideoCamera Node ready! Publishing H.265 (byte-stream)");
    }

    ~VideoCamera() {
        if (gst_pipeline_) {
            gst_element_set_state(gst_pipeline_, GST_STATE_NULL);
            gst_object_unref(gst_pipeline_);
        }
        RCLCPP_INFO(this->get_logger(), "VideoCamera Node shutdown");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    GstElement* gst_pipeline_;
    GstAppSink* gst_appsink_;

    unsigned int frame_count_;
    rclcpp::Time last_log_time_;

    static GstFlowReturn on_new_sample(GstAppSink* sink, gpointer user_data) {
        VideoCamera* node = static_cast<VideoCamera*>(user_data);
        return node->process_gst_sample(sink);
    }

    GstFlowReturn process_gst_sample(GstAppSink* sink) {
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

        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame";
        msg->encoding = "h265";
        msg->data.assign(map.data, map.data + map.size);

        publisher_->publish(*msg);

        frame_count_++;
        if ((this->now() - last_log_time_).seconds() >= 1.0) {
            RCLCPP_INFO(this->get_logger(), "[H.265] FPS: %d, Size: %.2f KB",
                        frame_count_, map.size / 1024.0);
            frame_count_ = 0;
            last_log_time_ = this->now();
        }

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        return GST_FLOW_OK;
    }
};

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