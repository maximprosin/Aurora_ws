#include <rclcpp/rclcpp.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <thread>

class TcpGimbalClient : public rclcpp::Node {
public:
    TcpGimbalClient() : Node("tcp_gimbal_client"), sock_(-1) {
        // Подключаемся к серверу
        connect_to_server("127.0.0.1", 5005);

        // Таймер для чтения данных из сокета
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TcpGimbalClient::read_data, this));
    }

    ~TcpGimbalClient() {
        if (sock_ != -1) {
            close(sock_);
        }
    }

private:
    void connect_to_server(const std::string& ip, int port) {
        sock_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Не удалось создать сокет");
            return;
        }

        struct sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr);

        if (connect(sock_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Не удалось подключиться к серверу");
            close(sock_);
            sock_ = -1;
        } else {
            RCLCPP_INFO(this->get_logger(), "Подключено к серверу %s:%d", ip.c_str(), port);
        }
    }

    void read_data() {
        if (sock_ == -1) return;

        char buffer[256];
        int bytes_read = read(sock_, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            std::string data(buffer);

            // Парсим данные: PITCH:-10.00,YAW:20.00
            float pitch = 0.0f, yaw = 0.0f;
            if (sscanf(data.c_str(), "PITCH:%f,YAW:%f", &pitch, &yaw) == 2) {
                RCLCPP_INFO(this->get_logger(), "Получена команда: Pitch=%.2f, Yaw=%.2f", pitch, yaw);
                // Здесь можно добавить логику управления другой камерой или запись в файл
            }
        }
    }

    int sock_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpGimbalClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
