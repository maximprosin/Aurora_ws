#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <errno.h>
#include <regex>
#include <sstream>
#include <map>

class TcpGimbalClient : public rclcpp::Node {
public:
    TcpGimbalClient() : Node("tcp_gimbal_client"), sock_(-1), reconnect_attempts_(0) {
        // ============================================
        // 1. ПАРАМЕТРЫ
        // ============================================
        this->declare_parameter("server_ip", "127.0.0.1");
        this->declare_parameter("server_port", 5005);
        this->declare_parameter("reconnect_delay_sec", 5.0);
        this->declare_parameter("max_reconnect_attempts", 10);
        this->declare_parameter("read_timeout_ms", 100);
        this->declare_parameter("publish_gimbal_topic", "/tcp_received/gimbal");
        this->declare_parameter("publish_teleop_topic", "/tcp_received/teleop");
        this->declare_parameter("publish_key_topic", "/tcp_received/key");

        server_ip_ = this->get_parameter("server_ip").as_string();
        server_port_ = this->get_parameter("server_port").as_int();
        reconnect_delay_sec_ = this->get_parameter("reconnect_delay_sec").as_double();
        max_reconnect_attempts_ = this->get_parameter("max_reconnect_attempts").as_int();
        read_timeout_ms_ = this->get_parameter("read_timeout_ms").as_int();
        publish_gimbal_topic_ = this->get_parameter("publish_gimbal_topic").as_string();
        publish_teleop_topic_ = this->get_parameter("publish_teleop_topic").as_string();
        publish_key_topic_ = this->get_parameter("publish_key_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "=== TCP-клиент запущен с параметрами ===");
        RCLCPP_INFO(this->get_logger(), "  server_ip: %s", server_ip_.c_str());
        RCLCPP_INFO(this->get_logger(), "  server_port: %d", server_port_);
        RCLCPP_INFO(this->get_logger(), "  publish_gimbal_topic: %s", publish_gimbal_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  publish_teleop_topic: %s", publish_teleop_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  publish_key_topic: %s", publish_key_topic_.c_str());

        // ============================================
        // 2. ПУБЛИКАТОРЫ
        // ============================================
        // Для камеры (углы)
        gimbal_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            publish_gimbal_topic_, 10);

        // Для тележки (скорости)
        teleop_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            publish_teleop_topic_, 10);

        // Для клавиш
        key_publisher_ = this->create_publisher<std_msgs::msg::String>(
            publish_key_topic_, 10);

        // ============================================
        // 3. ПОДКЛЮЧЕНИЕ
        // ============================================
        bool connected = connect_to_server(server_ip_, server_port_);
        if (!connected) {
            RCLCPP_ERROR(this->get_logger(), "Не удалось подключиться при старте. Буду пытаться переподключиться...");
            start_reconnect_timer();
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(read_timeout_ms_),
            std::bind(&TcpGimbalClient::read_data, this));

        // ============================================
        // 4. СТАТИСТИКА
        // ============================================
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&TcpGimbalClient::print_stats, this));
        command_count_ = 0;
        last_activity_time_ = this->now();
    }

    ~TcpGimbalClient() {
        if (sock_ != -1) {
            close(sock_);
        }
        RCLCPP_INFO(this->get_logger(), "🛑 Клиент остановлен. Всего команд: %d", command_count_);
    }

private:
    // ============================================
    // ПОДКЛЮЧЕНИЕ К СЕРВЕРУ
    // ============================================
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

        int result = connect(sock_, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
        if (result < 0 && errno != EINPROGRESS) {
            RCLCPP_WARN(this->get_logger(), "Не удалось подключиться: %s", strerror(errno));
            close(sock_);
            sock_ = -1;
            return false;
        }

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

    // ============================================
    // ПАРСИНГ И ПУБЛИКАЦИЯ
    // ============================================
    void parse_and_publish(const std::string& line) {
        std::string data = line;
        data.erase(std::remove(data.begin(), data.end(), '\r'), data.end());
        data.erase(std::remove(data.begin(), data.end(), '\n'), data.end());

        if (data.empty()) return;

        RCLCPP_DEBUG(this->get_logger(), "Парсинг: %s", data.c_str());

        // ============================================
        // 1. ПАРСИНГ КАМЕРЫ: CAMERA:pitch,yaw
        // ============================================
        float pitch, yaw;
        if (sscanf(data.c_str(), "CAMERA:%f,%f", &pitch, &yaw) == 2) {
            RCLCPP_INFO(this->get_logger(), "📷 Камера: Pitch=%.2f, Yaw=%.2f", pitch, yaw);
            command_count_++;
            last_activity_time_ = this->now();

            auto msg = std_msgs::msg::Float32MultiArray();
            msg.data.clear();
            msg.data.push_back(pitch);
            msg.data.push_back(yaw);
            gimbal_publisher_->publish(msg);
            return;
        }

        // ============================================
        // 2. ПАРСИНГ ТЕЛЕЖКИ: TELEOP:linear,angular
        // ============================================
        float linear, angular;
        if (sscanf(data.c_str(), "TELEOP:%f,%f", &linear, &angular) == 2) {
            RCLCPP_INFO(this->get_logger(), "🚜 Тележка: linear=%.2f, angular=%.2f", linear, angular);
            command_count_++;
            last_activity_time_ = this->now();

            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = linear;
            msg.angular.z = angular;
            teleop_publisher_->publish(msg);
            return;
        }

        // ============================================
        // 3. ПАРСИНГ КЛАВИШ: KEY:key_name
        // ============================================
        if (data.rfind("KEY:", 0) == 0) {
            std::string key_name = data.substr(4);

            std::map<std::string, std::string> key_names = {
                {"W", "Вперёд"},
                {"S", "Назад"},
                {"A", "Налево"},
                {"D", "Направо"},
                {"Q", "Разворот налево"},
                {"E", "Разворот направо"},
                {"SPACE", "СТОП/Центр камеры"},
                {"RELEASE", "Клавиша отпущена"},
                {"UP", "Наклон вверх"},
                {"DOWN", "Наклон вниз"},
                {"LEFT", "Поворот налево"},
                {"RIGHT", "Поворот направо"},
                {"PLUS", "Приближение"},
                {"MINUS", "Отдаление"},
                {"Z", "Зум приближение"},
                {"X", "Зум отдаление"},
                {"C", "Стоп зум"},
                {"H", "Справка"}
            };

            std::string display_name = "Неизвестная клавиша";
            auto it = key_names.find(key_name);
            if (it != key_names.end()) {
                display_name = it->second;
            }

            RCLCPP_INFO(this->get_logger(), "⌨️ Клавиша: [%s] -> %s", key_name.c_str(), display_name.c_str());
            command_count_++;
            last_activity_time_ = this->now();

            auto msg = std_msgs::msg::String();
            msg.data = key_name;
            key_publisher_->publish(msg);
            return;
        }

        // ============================================
        // 4. СТАРЫЙ ФОРМАТ ДЛЯ СОВМЕСТИМОСТИ
        // ============================================
        if (sscanf(data.c_str(), "PITCH:%f,YAW:%f", &pitch, &yaw) == 2) {
            RCLCPP_INFO(this->get_logger(), "📷 Камера (старый формат): Pitch=%.2f, Yaw=%.2f", pitch, yaw);
            command_count_++;
            last_activity_time_ = this->now();

            auto msg = std_msgs::msg::Float32MultiArray();
            msg.data.clear();
            msg.data.push_back(pitch);
            msg.data.push_back(yaw);
            gimbal_publisher_->publish(msg);
            return;
        }

        if (sscanf(data.c_str(), "LINEAR:%f,ANGULAR:%f", &linear, &angular) == 2) {
            RCLCPP_INFO(this->get_logger(), "🚜 Тележка (старый формат): linear=%.2f, angular=%.2f", linear, angular);
            command_count_++;
            last_activity_time_ = this->now();

            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = linear;
            msg.angular.z = angular;
            teleop_publisher_->publish(msg);
            return;
        }

        // ============================================
        // 5. НЕИЗВЕСТНЫЙ ФОРМАТ
        // ============================================
        RCLCPP_WARN(this->get_logger(), "⚠️ Неизвестный формат: %s", data.c_str());
    }

    // ============================================
    // ЧТЕНИЕ ДАННЫХ ИЗ СОКЕТА
    // ============================================
    void read_data() {
        if (sock_ == -1) {
            if (!reconnect_timer_ || !reconnect_timer_->is_canceled()) {
                start_reconnect_timer();
            }
            return;
        }

        char buffer[1024];
        ssize_t bytes_read = read(sock_, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            std::string data(buffer);

            RCLCPP_DEBUG(this->get_logger(), "Получено %zd байт", bytes_read);

            buffer_.append(data);

            size_t pos;
            while ((pos = buffer_.find('\n')) != std::string::npos) {
                std::string line = buffer_.substr(0, pos);
                buffer_.erase(0, pos + 1);

                if (!line.empty()) {
                    parse_and_publish(line);
                }
            }

            if (buffer_.size() > 4096) {
                RCLCPP_WARN(this->get_logger(), "Буфер переполнен, очищаем");
                buffer_.clear();
            }

        } else if (bytes_read == 0) {
            RCLCPP_WARN(this->get_logger(), "Сервер закрыл соединение");
            if (sock_ != -1) {
                close(sock_);
                sock_ = -1;
            }
            start_reconnect_timer();
        } else if (bytes_read == -1) {
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

    // ============================================
    // ПЕРЕПОДКЛЮЧЕНИЕ
    // ============================================
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

    // ============================================
    // СТАТИСТИКА
    // ============================================
    void print_stats() {
        auto now = this->now();
        double dt = (now - last_activity_time_).seconds();

        RCLCPP_INFO(this->get_logger(), "📊 СТАТИСТИКА:");
        RCLCPP_INFO(this->get_logger(), "   Всего команд: %d", command_count_);
        RCLCPP_INFO(this->get_logger(), "   Соединение: %s", (sock_ != -1) ? "✅ активно" : "❌ разорвано");
        RCLCPP_INFO(this->get_logger(), "   Активность: %.0f сек назад", dt);
    }

    // ============================================
    // ПЕРЕМЕННЫЕ
    // ============================================
    std::string server_ip_;
    int server_port_;
    double reconnect_delay_sec_;
    int max_reconnect_attempts_;
    int read_timeout_ms_;
    std::string publish_gimbal_topic_;
    std::string publish_teleop_topic_;
    std::string publish_key_topic_;

    int sock_;
    int reconnect_attempts_;
    std::string buffer_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gimbal_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr key_publisher_;

    rclcpp::Time last_activity_time_;
    int command_count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpGimbalClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
