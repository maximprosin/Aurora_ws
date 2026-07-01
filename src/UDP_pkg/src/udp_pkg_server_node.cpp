#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <boost/asio.hpp>
#include <vector>
#include <chrono>
#include <thread>

namespace io = boost::asio;
using io::ip::udp;

class UdpServerNode : public rclcpp::Node {
public:
    UdpServerNode() :
        Node("udp_server_node"),
        socket_(io_context_),
        frame_count_(0)
    {
        std::string client_ip = this->declare_parameter("pc_ip", "127.0.0.1");
        int udp_port = this->declare_parameter("udp_port", 12346);

        RCLCPP_INFO(this->get_logger(), "=== UDP H.265 SERVER ===");
        RCLCPP_INFO(this->get_logger(), "Client IP: %s", client_ip.c_str());
        RCLCPP_INFO(this->get_logger(), "UDP Port: %d", udp_port);

        try {
            socket_.open(udp::v4());
            client_endpoint_ = udp::endpoint(
                io::ip::address::from_string(client_ip),
                udp_port
                );
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "UDP error: %s", e.what());
            throw;
        }

        // ★ ПОДПИСКА НА H.265 ★
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_image",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            std::bind(&UdpServerNode::image_callback, this, std::placeholders::_1)
            );

        io_thread_ = std::thread([this]() {
            io_context_.run();
        });

        RCLCPP_INFO(this->get_logger(), "UDP Server ready!");
    }

    ~UdpServerNode() {
        io_context_.stop();
        if (io_thread_.joinable()) io_thread_.join();
        if (socket_.is_open()) {
            boost::system::error_code ec;
            socket_.close(ec);
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    io::io_context io_context_;
    udp::socket socket_;
    udp::endpoint client_endpoint_;
    std::thread io_thread_;

    unsigned int frame_count_;
    rclcpp::Time last_log_time_ = this->now();

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (msg->data.empty()) return;

        // ★ ОТПРАВЛЯЕМ H.265 ПО UDP ★
        socket_.async_send_to(
            io::buffer(msg->data.data(), msg->data.size()),
            client_endpoint_,
            [this](const boost::system::error_code& ec, size_t) {
                if (ec) {
                    RCLCPP_ERROR(this->get_logger(), "UDP error: %s", ec.message().c_str());
                }
            }
            );

        frame_count_++;
        if ((this->now() - last_log_time_).seconds() >= 1.0) {
            RCLCPP_INFO(this->get_logger(),
                        "[UDP] FPS: %d, Size: %.2f KB",
                        frame_count_, msg->data.size() / 1024.0);
            frame_count_ = 0;
            last_log_time_ = this->now();
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UdpServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}