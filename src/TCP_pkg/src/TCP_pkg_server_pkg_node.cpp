#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/gimbal_manager_set_pitchyaw.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>        // Для fcntl
#include <thread>
#include <vector>
#include <mutex>
#include <cstring>
#include <string>
#include <errno.h>
#include <atomic>

class TcpGimbalServer : public rclcpp::Node {
public:
    TcpGimbalServer() : Node("tcp_gimbal_server"), server_fd_(-1), running_(true) {
        // Параметры
        this->declare_parameter("server_ip", "0.0.0.0");  // Слушаем все интерфейсы
        this->declare_parameter("server_port", 5005);
        this->declare_parameter("max_clients", 10);

        server_ip_ = this->get_parameter("server_ip").as_string();
        server_port_ = this->get_parameter("server_port").as_int();
        max_clients_ = this->get_parameter("max_clients").as_int();

        // Подписка на команды
        subscription_ = this->create_subscription<mavros_msgs::msg::GimbalManagerSetPitchyaw>(
            "/gimbal/commands", 10,
            std::bind(&TcpGimbalServer::gimbal_callback, this, std::placeholders::_1));

        angle_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/server_received/angles", 10);

        // Запуск сервера
        if (!start_server(server_ip_, server_port_)) {
            RCLCPP_ERROR(this->get_logger(), "Не удалось запустить сервер");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "✅ TCP-сервер запущен на %s:%d", server_ip_.c_str(), server_port_);
        RCLCPP_INFO(this->get_logger(), "📋 Ожидание команд в топике /gimbal/commands");
    }

    ~TcpGimbalServer() {
        running_ = false;
        if (accept_thread_.joinable()) {
            accept_thread_.join();
        }

        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            for (int client : clients_) {
                if (client != -1) {
                    close(client);
                }
            }
            clients_.clear();
        }

        if (server_fd_ != -1) {
            close(server_fd_);
        }
    }

private:
    bool start_server(const std::string& ip, int port) {
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Не удалось создать сокет: %s", strerror(errno));
            return false;
        }

        int opt = 1;
        if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            RCLCPP_WARN(this->get_logger(), "Не удалось установить SO_REUSEADDR: %s", strerror(errno));
        }

        struct sockaddr_in address;
        memset(&address, 0, sizeof(address));
        address.sin_family = AF_INET;
        address.sin_port = htons(port);

        if (inet_pton(AF_INET, ip.c_str(), &address.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Неверный IP-адрес: %s", ip.c_str());
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка привязки к %s:%d: %s", ip.c_str(), port, strerror(errno));
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        if (listen(server_fd_, max_clients_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка прослушивания порта: %s", strerror(errno));
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        // 🟢 Устанавливаем неблокирующий режим для серверного сокета
        int flags = fcntl(server_fd_, F_GETFL, 0);
        if (flags == -1 || fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK) == -1) {
            RCLCPP_WARN(this->get_logger(), "Не удалось установить неблокирующий режим для серверного сокета");
        }

        accept_thread_ = std::thread(&TcpGimbalServer::accept_clients, this);
        return true;
    }

    void accept_clients() {
        while (running_ && rclcpp::ok()) {
            struct sockaddr_in client_addr;
            socklen_t addrlen = sizeof(client_addr);

            int client_socket = accept(server_fd_, (struct sockaddr*)&client_addr, &addrlen);

            if (client_socket < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                if (errno != EINTR) {
                    RCLCPP_WARN(this->get_logger(), "Ошибка accept(): %s", strerror(errno));
                }
                continue;
            }

            // 🟢 Устанавливаем неблокирующий режим для клиентского сокета
            int flags = fcntl(client_socket, F_GETFL, 0);
            if (flags != -1) {
                fcntl(client_socket, F_SETFL, flags | O_NONBLOCK);
            }

            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                if (clients_.size() >= static_cast<size_t>(max_clients_)) {
                    RCLCPP_WARN(this->get_logger(), "Достигнут лимит клиентов. Отклоняем.");
                    close(client_socket);
                    continue;
                }
            }

            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);

            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                clients_.push_back(client_socket);
                RCLCPP_INFO(this->get_logger(), "✅ Новый клиент: %s (всего: %zu)", client_ip, clients_.size());
            }
        }
    }

    void publish_received_angles(float pitch, float yaw) {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data.clear();
        msg.data.push_back(pitch);
        msg.data.push_back(yaw);
        angle_publisher_->publish(msg);
    }

    void gimbal_callback(const mavros_msgs::msg::GimbalManagerSetPitchyaw::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "📥 Получена команда: Pitch=%.2f, Yaw=%.2f", msg->pitch, msg->yaw);
        publish_received_angles(msg->pitch, msg->yaw);

        char buffer[256];
        int len = snprintf(buffer, sizeof(buffer), "PITCH:%.2f,YAW:%.2f\n", msg->pitch, msg->yaw);

        if (len < 0 || len >= static_cast<int>(sizeof(buffer))) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка форматирования сообщения");
            return;
        }

        std::lock_guard<std::mutex> lock(clients_mutex_);

        if (clients_.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Нет клиентов для отправки");
            return;
        }

        for (auto it = clients_.begin(); it != clients_.end();) {
            int client = *it;
            ssize_t bytes_sent = send(client, buffer, strlen(buffer), MSG_NOSIGNAL | MSG_DONTWAIT);

            if (bytes_sent < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    RCLCPP_WARN(this->get_logger(), "Клиент отключился (send: %s). Удаляем.", strerror(errno));
                    close(client);
                    it = clients_.erase(it);
                } else {
                    ++it;
                }
            } else {
                RCLCPP_DEBUG(this->get_logger(), "✅ Отправлено %zd байт", bytes_sent);
                ++it;
            }
        }
    }

    std::string server_ip_;
    int server_port_;
    int max_clients_;

    int server_fd_;
    std::vector<int> clients_;
    std::mutex clients_mutex_;
    std::thread accept_thread_;
    std::atomic<bool> running_;

    rclcpp::Subscription<mavros_msgs::msg::GimbalManagerSetPitchyaw>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr angle_publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpGimbalServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}