#include "rclcpp/rclcpp.hpp"
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <boost/asio.hpp>
#include <array>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

namespace io = boost::asio;
using io::ip::udp;

class UdpClientNode : public rclcpp::Node {
public:
    UdpClientNode() :
        Node("udp_client_node"),
        socket_(io_context_, udp::endpoint(udp::v4(), 12346))
    {
        RCLCPP_INFO(this->get_logger(), "=== UDP H.265 CLIENT ===");
        RCLCPP_INFO(this->get_logger(), "Listening on port: 12346");

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
            RCLCPP_ERROR(this->get_logger(), "GStreamer error: %s",
                         error ? error->message : "unknown");
            if (error) g_error_free(error);
            return;
        }

        GstElement* src_elem = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
        appsrc_ = GST_APP_SRC(src_elem);
        gst_object_unref(src_elem);

        GstElement* sink_elem = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        appsink_ = GST_APP_SINK(sink_elem);
        gst_object_unref(sink_elem);

        GstCaps* caps = gst_caps_new_simple("video/x-h265",
                                            "stream-format", G_TYPE_STRING, "byte-stream",
                                            nullptr);
        gst_app_src_set_caps(appsrc_, caps);
        gst_caps_unref(caps);
        gst_app_src_set_size(appsrc_, -1);
        gst_app_src_set_stream_type(appsrc_, GST_APP_STREAM_TYPE_STREAM);

        gst_app_sink_set_emit_signals(appsink_, true);
        gst_app_sink_set_max_buffers(appsink_, 1);
        gst_app_sink_set_drop(appsink_, true);

        g_signal_connect(appsink_, "new-sample",
                         G_CALLBACK(on_new_sample), this);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        start_receive();

        io_thread_ = std::thread([this]() {
            io_context_.run();
        });

        RCLCPP_INFO(this->get_logger(), "UDP Client ready!");
    }

    ~UdpClientNode() {
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
    }

private:
    io::io_context io_context_;
    udp::socket socket_;
    udp::endpoint remote_endpoint_;
    std::thread io_thread_;

    GstElement* pipeline_;
    GstAppSrc* appsrc_;
    GstAppSink* appsink_;

    std::array<uint8_t, 65536> recv_buffer_;
    rclcpp::Time last_log_time_ = this->now();
    unsigned int frame_count_ = 0;

    void start_receive() {
        socket_.async_receive_from(
            io::buffer(recv_buffer_),
            remote_endpoint_,
            [this](const boost::system::error_code& error, size_t bytes_transferred) {
                handle_receive(error, bytes_transferred);
            }
            );
    }

    void handle_receive(const boost::system::error_code& error,
                        size_t bytes_transferred) {
        if (!error && bytes_transferred > 0) {
            GstBuffer* buffer = gst_buffer_new_allocate(nullptr, bytes_transferred, nullptr);

            GstMapInfo map;
            gst_buffer_map(buffer, &map, GST_MAP_WRITE);
            memcpy(map.data, recv_buffer_.data(), bytes_transferred);
            gst_buffer_unmap(buffer, &map);

            GST_BUFFER_PTS(buffer) = GST_CLOCK_TIME_NONE;
            GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
            GST_BUFFER_DURATION(buffer) = GST_CLOCK_TIME_NONE;

            gst_app_src_push_buffer(appsrc_, buffer);

            frame_count_++;
            if ((this->now() - last_log_time_).seconds() >= 1.0) {
                RCLCPP_INFO(this->get_logger(), "[FPS] %d", frame_count_);
                frame_count_ = 0;
                last_log_time_ = this->now();
            }
        }
        start_receive();
    }

    static GstFlowReturn on_new_sample(GstAppSink* sink, gpointer user_data) {
        auto* node = static_cast<UdpClientNode*>(user_data);
        return node->process_sample(sink);
    }

    GstFlowReturn process_sample(GstAppSink* sink) {
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

        int width = 0, height = 0;
        gst_structure_get_int(structure, "width", &width);
        gst_structure_get_int(structure, "height", &height);

        if (width > 0 && height > 0 && map.size > 0) {
            cv::Mat frame(height, width, CV_8UC3, map.data);
            if (!frame.empty()) {
                cv::imshow("UDP Video", frame);
                cv::waitKey(1);
            }
        }

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        return GST_FLOW_OK;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UdpClientNode>();

    std::thread ros_thread([&]() {
        rclcpp::spin(node);
    });

    while (rclcpp::ok()) {
        cv::waitKey(10);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ros_thread.join();
    rclcpp::shutdown();
    return 0;
}