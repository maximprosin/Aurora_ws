#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <boost/asio.hpp>
#include <thread>

namespace io = boost::asio;
using io::ip::udp;

/**
 * @brief UDP-сервер для передачи H.265 видео на ПК.
 * Подписывается на топик /camera_image и отправляет кадры по UDP.
 */
class UdpServerNode : public rclcpp::Node {
public:
    UdpServerNode() :
        Node("udp_server_node"),
        socket_(io_context_),
        frame_count_(0),
        last_log_time_(this->now())
    {
        RCLCPP_INFO(this->get_logger(), "=== UDP H.265 SERVER ===");

        // --- 1. Параметры ---
        std::string client_ip = this->declare_parameter("pc_ip", "127.0.0.1");
        int udp_port = this->declare_parameter("udp_port", 12346);

        // Валидация порта
        if (udp_port < 1 || udp_port > 65535) {
            RCLCPP_WARN(this->get_logger(), "Invalid UDP port %d, using default 12346", udp_port);
            udp_port = 12346;
        }

        RCLCPP_INFO(this->get_logger(), "Client IP: %s", client_ip.c_str());
        RCLCPP_INFO(this->get_logger(), "UDP Port: %d", udp_port);

        // --- 2. Открытие сокета ---
        try {
            socket_.open(udp::v4());
            client_endpoint_ = udp::endpoint(
                io::ip::address::from_string(client_ip),
                udp_port
                );
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "UDP initialization error: %s", e.what());
            throw;  // Критическая ошибка — узел не может работать
        }

        // --- 3. Подписка на топик /camera_image ---
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_image",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            std::bind(&UdpServerNode::image_callback, this, std::placeholders::_1)
            );

        // --- 4. Запуск потока для io_context ---
        io_thread_ = std::thread([this]() {
            io_context_.run();
        });

        RCLCPP_INFO(this->get_logger(), "UDP Server ready!");
    }

    ~UdpServerNode() {
        // Остановка io_context и ожидание завершения потока
        io_context_.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
        // Закрытие сокета
        if (socket_.is_open()) {
            boost::system::error_code ec;
            socket_.close(ec);
            if (ec) {
                RCLCPP_WARN(this->get_logger(), "Error closing UDP socket: %s", ec.message().c_str());
            }
        }
        RCLCPP_INFO(this->get_logger(), "UDP Server shutdown");
    }

private:
    // --- Члены класса ---
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    io::io_context io_context_;
    udp::socket socket_;
    udp::endpoint client_endpoint_;
    std::thread io_thread_;

    unsigned int frame_count_;
    rclcpp::Time last_log_time_;

    // --- Колбэк на новый кадр ---
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Защита от пустых сообщений
        if (msg->data.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1.0,
                                 "Received empty image, skipping");
            return;
        }

        // Асинхронная отправка по UDP
        socket_.async_send_to(
            io::buffer(msg->data.data(), msg->data.size()),
            client_endpoint_,
            [this](const boost::system::error_code& ec, size_t bytes_sent) {
                if (ec) {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1.0,
                                          "UDP send error: %s", ec.message().c_str());
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "Sent %zu bytes", bytes_sent);
                }
            }
            );

        // Статистика
        frame_count_++;
        auto now = this->now();
        if ((now - last_log_time_).seconds() >= 1.0) {
            RCLCPP_INFO(this->get_logger(),
                        "[UDP Server] FPS: %d, Frame size: %.2f KB",
                        frame_count_, msg->data.size() / 1024.0);
            frame_count_ = 0;
            last_log_time_ = now;
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