#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/gimbal_manager_set_pitchyaw.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <vector>
#include <mutex>
#include <cstring>
#include <string>

class TcpGimbalServer : public rclcpp::Node {
public:
    TcpGimbalServer() : Node("tcp_gimbal_server"), server_fd_(-1) {
        // Подписываемся на топик с командами камеры
        subscription_ = this->create_subscription<mavros_msgs::msg::GimbalManagerSetPitchyaw>(
            "/gimbal/commands", 10,
            std::bind(&TcpGimbalServer::gimbal_callback, this, std::placeholders::_1));

        // ЖЕСТКО ЗАДАЕМ IP-АДРЕС И ПОРТ ДЛЯ ПРОСЛУШИВАНИЯ
        std::string server_ip = "127.0.0.1";  // ← ЗДЕСЬ ВАШ IP
        int server_port = 5005;

        start_server(server_ip, server_port);
        RCLCPP_INFO(this->get_logger(), "TCP-сервер запущен на %s:%d",
                    server_ip.c_str(), server_port);
    }

    ~TcpGimbalServer() {
        if (server_fd_ != -1) {
            close(server_fd_);
        }
    }

private:
    void start_server(const std::string& ip, int port) {
        // Создаем сокет
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Не удалось создать сокет");
            return;
        }

        // Разрешаем переиспользование адреса
        int opt = 1;
        if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            RCLCPP_WARN(this->get_logger(), "Не удалось установить SO_REUSEADDR");
        }

        // Настраиваем адрес сервера с КОНКРЕТНЫМ IP
        struct sockaddr_in address;
        memset(&address, 0, sizeof(address));
        address.sin_family = AF_INET;
        address.sin_port = htons(port);

        // Преобразуем строковый IP в бинарный формат
        if (inet_pton(AF_INET, ip.c_str(), &address.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Неверный IP-адрес: %s", ip.c_str());
            close(server_fd_);
            server_fd_ = -1;
            return;
        }

        // Привязываем сокет к конкретному IP и порту
        if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка привязки к %s:%d", ip.c_str(), port);
            close(server_fd_);
            server_fd_ = -1;
            return;
        }

        // Начинаем прослушивание
        if (listen(server_fd_, 3) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка прослушивания порта");
            close(server_fd_);
            server_fd_ = -1;
            return;
        }

        // Запускаем поток для приема подключений
        accept_thread_ = std::thread(&TcpGimbalServer::accept_clients, this);
    }

    void accept_clients() {
        while (rclcpp::ok()) {
            struct sockaddr_in client_addr;
            socklen_t addrlen = sizeof(client_addr);
            int client_socket = accept(server_fd_, (struct sockaddr*)&client_addr, &addrlen);

            if (client_socket < 0) {
                if (errno != EINTR) {
                    RCLCPP_WARN(this->get_logger(), "Ошибка accept()");
                }
                continue;
            }

            // Получаем IP клиента для логирования
            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
            RCLCPP_INFO(this->get_logger(), "Новый клиент подключен: %s", client_ip);

            std::lock_guard<std::mutex> lock(clients_mutex_);
            clients_.push_back(client_socket);
        }
    }

    void gimbal_callback(const mavros_msgs::msg::GimbalManagerSetPitchyaw::SharedPtr msg) {
        // Формируем сообщение для отправки клиентам
        char buffer[256];
        snprintf(buffer, sizeof(buffer), "PITCH:%.2f,YAW:%.2f\n", msg->pitch, msg->yaw);

        std::lock_guard<std::mutex> lock(clients_mutex_);
        // Отправляем данные всем клиентам
        for (auto it = clients_.begin(); it != clients_.end();) {
            int client = *it;
            if (send(client, buffer, strlen(buffer), 0) < 0) {
                RCLCPP_WARN(this->get_logger(), "Клиент отключился, удаляем");
                close(client);
                it = clients_.erase(it);
            } else {
                ++it;
            }
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::GimbalManagerSetPitchyaw>::SharedPtr subscription_;
    std::vector<int> clients_;
    std::mutex clients_mutex_;
    int server_fd_;
    std::thread accept_thread_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpGimbalServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
