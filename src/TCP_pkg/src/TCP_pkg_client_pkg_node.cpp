#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>        // Для fcntl
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <errno.h>

class TcpGimbalClient : public rclcpp::Node {
public:
    TcpGimbalClient() : Node("tcp_gimbal_client"), sock_(-1), reconnect_attempts_(0) {
        // Параметры
        this->declare_parameter("server_ip", "127.0.0.1");
        this->declare_parameter("server_port", 5005);
        this->declare_parameter("reconnect_delay_sec", 5.0);
        this->declare_parameter("max_reconnect_attempts", 10);
        this->declare_parameter("read_timeout_ms", 100);  // Увеличен таймаут
        this->declare_parameter("publish_topic", "/tcp_received/angles");

        server_ip_ = this->get_parameter("server_ip").as_string();
        server_port_ = this->get_parameter("server_port").as_int();
        reconnect_delay_sec_ = this->get_parameter("reconnect_delay_sec").as_double();
        max_reconnect_attempts_ = this->get_parameter("max_reconnect_attempts").as_int();
        read_timeout_ms_ = this->get_parameter("read_timeout_ms").as_int();
        publish_topic_ = this->get_parameter("publish_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "=== TCP-клиент запущен с параметрами ===");
        RCLCPP_INFO(this->get_logger(), "  server_ip: %s", server_ip_.c_str());
        RCLCPP_INFO(this->get_logger(), "  server_port: %d", server_port_);
        RCLCPP_INFO(this->get_logger(), "  publish_topic: %s", publish_topic_.c_str());

        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(publish_topic_, 10);

        // Подключаемся
        bool connected = connect_to_server(server_ip_, server_port_);
        if (!connected) {
            RCLCPP_ERROR(this->get_logger(), "Не удалось подключиться при старте. Буду пытаться переподключиться...");
            start_reconnect_timer();
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(read_timeout_ms_),
            std::bind(&TcpGimbalClient::read_data, this));
    }

    ~TcpGimbalClient() {
        if (sock_ != -1) {
            close(sock_);
        }
    }

private:
    bool connect_to_server(const std::string& ip, int port) {
        if (sock_ != -1) {
            close(sock_);
            sock_ = -1;
        }

        sock_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка создания сокета: %s", strerror(errno));
            return false;
        }

        // 🟢 ВАЖНО: Устанавливаем НЕБЛОКИРУЮЩИЙ режим для сокета
        int flags = fcntl(sock_, F_GETFL, 0);
        if (flags == -1) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка получения флагов сокета");
            close(sock_);
            sock_ = -1;
            return false;
        }
        if (fcntl(sock_, F_SETFL, flags | O_NONBLOCK) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка установки неблокирующего режима");
            close(sock_);
            sock_ = -1;
            return false;
        }

        struct sockaddr_in serv_addr;
        memset(&serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);

        if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Неверный IP-адрес: %s", ip.c_str());
            close(sock_);
            sock_ = -1;
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Попытка подключения к %s:%d...", ip.c_str(), port);

        // 🟢 Для неблокирующего сокета connect может вернуть EINPROGRESS
        int result = connect(sock_, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
        if (result < 0 && errno != EINPROGRESS) {
            RCLCPP_WARN(this->get_logger(), "Не удалось подключиться: %s", strerror(errno));
            close(sock_);
            sock_ = -1;
            return false;
        }

        // Проверяем успешность подключения с таймаутом
        fd_set fdset;
        FD_ZERO(&fdset);
        FD_SET(sock_, &fdset);
        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        if (select(sock_ + 1, NULL, &fdset, NULL, &tv) == 1) {
            int so_error;
            socklen_t len = sizeof(so_error);
            getsockopt(sock_, SOL_SOCKET, SO_ERROR, &so_error, &len);
            if (so_error == 0) {
                RCLCPP_INFO(this->get_logger(), "✅ Подключено к серверу %s:%d", ip.c_str(), port);
                reconnect_attempts_ = 0;
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Ошибка подключения: %s", strerror(so_error));
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Таймаут подключения к серверу");
        }

        close(sock_);
        sock_ = -1;
        return false;
    }

    void read_data() {
        if (sock_ == -1) {
            if (!reconnect_timer_ || !reconnect_timer_->is_canceled()) {
                start_reconnect_timer();
            }
            return;
        }

        char buffer[256];
        ssize_t bytes_read = read(sock_, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            std::string data(buffer);

            // Удаляем символ новой строки, если есть
            if (!data.empty() && data.back() == '\n') {
                data.pop_back();
            }

            RCLCPP_DEBUG(this->get_logger(), "Получены данные: %s", data.c_str());

            float pitch = 0.0f, yaw = 0.0f;
            if (sscanf(data.c_str(), "PITCH:%f,YAW:%f", &pitch, &yaw) == 2) {
                RCLCPP_INFO(this->get_logger(), "📦 Получена команда: Pitch=%.2f, Yaw=%.2f", pitch, yaw);
                process_gimbal_command(pitch, yaw);
                publish_received_angles(pitch, yaw);
            } else {
                RCLCPP_WARN(this->get_logger(), "Не удалось распарсить данные: %s", data.c_str());
            }
        } else if (bytes_read == 0) {
            RCLCPP_WARN(this->get_logger(), "Сервер закрыл соединение");
            if (sock_ != -1) {
                close(sock_);
                sock_ = -1;
            }
            start_reconnect_timer();
        } else if (bytes_read == -1) {
            // EAGAIN/EWOULDBLOCK - это нормально для неблокирующего сокета
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(this->get_logger(), "Ошибка чтения из сокета: %s", strerror(errno));
                if (sock_ != -1) {
                    close(sock_);
                    sock_ = -1;
                }
                start_reconnect_timer();
            }
        }
    }

    void start_reconnect_timer() {
        if (reconnect_timer_ && !reconnect_timer_->is_canceled()) {
            return;
        }
        reconnect_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(reconnect_delay_sec_ * 1000)),
            std::bind(&TcpGimbalClient::reconnect_callback, this));
    }

    void reconnect_callback() {
        reconnect_attempts_++;
        if (reconnect_attempts_ > max_reconnect_attempts_) {
            RCLCPP_ERROR(this->get_logger(), "❌ Превышено максимальное число попыток переподключения");
            reconnect_timer_->cancel();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Попытка переподключения #%d...", reconnect_attempts_);
        if (connect_to_server(server_ip_, server_port_)) {
            RCLCPP_INFO(this->get_logger(), "✅ Переподключение успешно!");
            reconnect_timer_->cancel();
        } else {
            RCLCPP_WARN(this->get_logger(), "Попытка переподключения #%d не удалась", reconnect_attempts_);
        }
    }

    void publish_received_angles(float pitch, float yaw) {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data.clear();
        msg.data.push_back(pitch);
        msg.data.push_back(yaw);
        publisher_->publish(msg);
    }

    void process_gimbal_command(float pitch, float yaw) {
        last_pitch_ = pitch;
        last_yaw_ = yaw;
        // Здесь можно добавить управление камерой
    }

    std::string server_ip_;
    int server_port_;
    double reconnect_delay_sec_;
    int max_reconnect_attempts_;
    int read_timeout_ms_;
    std::string publish_topic_;

    int sock_;
    int reconnect_attempts_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

    float last_pitch_ = 0.0f;
    float last_yaw_ = 0.0f;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpGimbalClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}