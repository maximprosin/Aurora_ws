#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <string>
#include <mutex>

class VideoCamera : public rclcpp::Node
{
public:
    VideoCamera() : Node("video_camera_node"),
        pipeline_(nullptr),
        appsink_(nullptr),
        frame_count_(0),
        last_log_time_(this->now()),
        current_stream_id_(0)
    {
        RCLCPP_INFO(this->get_logger(), "=== VideoCamera Node Started ===");

        // --- 1. Параметры URL для двух потоков ---
        this->declare_parameter<std::string>("rtsp_url_0", "rtsp://192.168.144.25:8554/video1");
        this->declare_parameter<std::string>("rtsp_url_1", "rtsp://192.168.144.25:8554/video2");
        rtsp_urls_[0] = this->get_parameter("rtsp_url_0").as_string();
        rtsp_urls_[1] = this->get_parameter("rtsp_url_1").as_string();

        RCLCPP_INFO(this->get_logger(), "RTSP URLs:");
        for (int i = 0; i < 2; ++i) {
            RCLCPP_INFO(this->get_logger(), "  [%d] %s", i, rtsp_urls_[i].c_str());
        }

        // --- 2. Издатель ---
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort();
        qos.durability_volatile();
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera_image", qos);

        // --- 3. Подписка на команды переключения потока ---
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/set_rtsp_stream", 10,
            std::bind(&VideoCamera::on_set_stream, this, std::placeholders::_1));

        // --- 4. Инициализация GStreamer ---
        gst_init(nullptr, nullptr);

        // --- 5. Запуск первого потока ---
        if (!build_and_start_pipeline(rtsp_urls_[0])) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start initial pipeline");
        } else {
            current_stream_id_ = 0;
        }

        RCLCPP_INFO(this->get_logger(), "VideoCamera Node ready!");
    }

    ~VideoCamera()
    {
        stop_pipeline();
        RCLCPP_INFO(this->get_logger(), "VideoCamera Node shutdown");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

    GstElement* pipeline_;
    GstAppSink* appsink_;
    std::mutex pipeline_mutex_;

    unsigned int frame_count_;
    rclcpp::Time last_log_time_;

    std::string rtsp_urls_[2];
    int current_stream_id_;

    void on_set_stream(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int new_id = msg->data;
        if (new_id < 0 || new_id > 1) {
            RCLCPP_WARN(this->get_logger(), "Invalid stream id: %d (must be 0,1)", new_id);
            return;
        }
        if (new_id == current_stream_id_) {
            RCLCPP_DEBUG(this->get_logger(), "Stream %d already active", new_id);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Switching to stream %d: %s", new_id, rtsp_urls_[new_id].c_str());

        stop_pipeline();

        if (build_and_start_pipeline(rtsp_urls_[new_id])) {
            current_stream_id_ = new_id;
            RCLCPP_INFO(this->get_logger(), "Successfully switched to stream %d", new_id);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch to stream %d, trying to recover", new_id);
            if (!build_and_start_pipeline(rtsp_urls_[current_stream_id_])) {
                RCLCPP_ERROR(this->get_logger(), "Failed to recover previous stream!");
            }
        }
    }

    void stop_pipeline()
    {
        std::lock_guard<std::mutex> lock(pipeline_mutex_);
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            appsink_ = nullptr;
            RCLCPP_DEBUG(this->get_logger(), "Pipeline stopped");
        }
    }

    bool build_and_start_pipeline(const std::string& url)
    {
        std::lock_guard<std::mutex> lock(pipeline_mutex_);

        if (pipeline_) {
            RCLCPP_WARN(this->get_logger(), "Pipeline already exists, stopping it first");
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            appsink_ = nullptr;
        }

        // --- ВОТ ЭТОТ ПАЙПЛАЙН РАБОТАЛ, ВОЗВРАЩАЮ ЕГО ---
        std::string pipeline_str =
            "rtspsrc location=" + url + " latency=0 ! "
                                        "rtph265depay ! "
                                        "h265parse ! "
                                        "capsfilter caps=video/x-h265,stream-format=byte-stream ! "
                                        "appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false";

        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        if (!pipeline_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline for URL %s: %s",
                         url.c_str(), error ? error->message : "unknown");
            if (error) g_error_free(error);
            return false;
        }

        GstElement* sink_elem = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        if (!sink_elem) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get 'sink' element");
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            return false;
        }
        appsink_ = GST_APP_SINK(sink_elem);
        gst_object_unref(sink_elem);

        gst_app_sink_set_emit_signals(appsink_, true);
        gst_app_sink_set_max_buffers(appsink_, 1);
        gst_app_sink_set_drop(appsink_, true);

        g_signal_connect(appsink_, "new-sample",
                         G_CALLBACK(VideoCamera::on_new_sample_static), this);

        GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set pipeline to PLAYING for URL %s", url.c_str());
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            appsink_ = nullptr;
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Pipeline started for URL: %s", url.c_str());
        return true;
    }

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