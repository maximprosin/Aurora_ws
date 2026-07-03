#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>  // Добавлено для публикации
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <errno.h>

class TcpGimbalClient : public rclcpp::Node {
public:
    TcpGimbalClient() : Node("tcp_gimbal_client"), sock_(-1), reconnect_attempts_(0) {
        // ============================================
        // 1. ПАРАМЕТРИЗАЦИЯ: Загружаем параметры
        // ============================================
        this->declare_parameter("server_ip", "127.0.0.1");
        this->declare_parameter("server_port", 5005);
        this->declare_parameter("reconnect_delay_sec", 5.0);
        this->declare_parameter("max_reconnect_attempts", 10);
        this->declare_parameter("read_timeout_ms", 50);
        this->declare_parameter("publish_topic", "/tcp_received/angles");  // Новый параметр

        server_ip_ = this->get_parameter("server_ip").as_string();
        server_port_ = this->get_parameter("server_port").as_int();
        reconnect_delay_sec_ = this->get_parameter("reconnect_delay_sec").as_double();
        max_reconnect_attempts_ = this->get_parameter("max_reconnect_attempts").as_int();
        read_timeout_ms_ = this->get_parameter("read_timeout_ms").as_int();
        publish_topic_ = this->get_parameter("publish_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "=== TCP-клиент запущен с параметрами ===");
        RCLCPP_INFO(this->get_logger(), "  server_ip: %s", server_ip_.c_str());
        RCLCPP_INFO(this->get_logger(), "  server_port: %d", server_port_);
        RCLCPP_INFO(this->get_logger(), "  reconnect_delay: %.1f сек", reconnect_delay_sec_);
        RCLCPP_INFO(this->get_logger(), "  max_reconnect_attempts: %d", max_reconnect_attempts_);
        RCLCPP_INFO(this->get_logger(), "  publish_topic: %s", publish_topic_.c_str());

        // ============================================
        // 2. ПУБЛИКАТОР
        // ============================================
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            publish_topic_, 10);

        // ============================================
        // 3. ИНИЦИАЛИЗАЦИЯ: Подключаемся к серверу
        // ============================================
        bool connected = connect_to_server(server_ip_, server_port_);
        if (!connected) {
            RCLCPP_ERROR(this->get_logger(), "Не удалось подключиться при старте. Буду пытаться переподключиться...");
            start_reconnect_timer();
        }

        // ============================================
        // 4. ТАЙМЕР ДЛЯ ЧТЕНИЯ ДАННЫХ
        // ============================================
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(read_timeout_ms_),
            std::bind(&TcpGimbalClient::read_data, this));
    }

    ~TcpGimbalClient() {
        if (sock_ != -1) {
            close(sock_);
            RCLCPP_DEBUG(this->get_logger(), "Сокет закрыт");
        }
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
            RCLCPP_ERROR(this->get_logger(), "Ошибка создания сокета: %s (errno=%d)",
                         strerror(errno), errno);
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

        RCLCPP_DEBUG(this->get_logger(), "Попытка подключения к %s:%d...", ip.c_str(), port);
        if (connect(sock_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            RCLCPP_WARN(this->get_logger(), "Не удалось подключиться к %s:%d: %s (errno=%d)",
                        ip.c_str(), port, strerror(errno), errno);
            close(sock_);
            sock_ = -1;
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "✅ Подключено к серверу %s:%d", ip.c_str(), port);
        reconnect_attempts_ = 0;
        return true;
    }

    // ============================================
    // ТАЙМЕР ДЛЯ ПЕРЕПОДКЛЮЧЕНИЯ
    // ============================================
    void start_reconnect_timer() {
        reconnect_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(reconnect_delay_sec_ * 1000)),
            std::bind(&TcpGimbalClient::reconnect_callback, this));
        RCLCPP_INFO(this->get_logger(), "Запущен таймер переподключения (интервал %.1f сек)",
                    reconnect_delay_sec_);
    }

    void reconnect_callback() {
        reconnect_attempts_++;
        if (reconnect_attempts_ > max_reconnect_attempts_) {
            RCLCPP_ERROR(this->get_logger(),
                         "❌ Превышено максимальное число попыток переподключения (%d). Отключаю таймер.",
                         max_reconnect_attempts_);
            reconnect_timer_->cancel();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Попытка переподключения #%d...", reconnect_attempts_);
        if (connect_to_server(server_ip_, server_port_)) {
            RCLCPP_INFO(this->get_logger(), "✅ Переподключение успешно!");
            reconnect_timer_->cancel();
        } else {
            RCLCPP_WARN(this->get_logger(), "Попытка переподключения #%d не удалась",
                        reconnect_attempts_);
        }
    }

    // ============================================
    // ПУБЛИКАЦИЯ ПОЛУЧЕННЫХ ДАННЫХ
    // ============================================
    void publish_received_angles(float pitch, float yaw) {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data.clear();
        msg.data.push_back(pitch);
        msg.data.push_back(yaw);
        publisher_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "📤 Опубликованы углы в топик %s: Pitch=%.2f, Yaw=%.2f",
                     publish_topic_.c_str(), pitch, yaw);
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

        char buffer[256];
        ssize_t bytes_read = read(sock_, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            std::string data(buffer);
            RCLCPP_DEBUG(this->get_logger(), "Получены данные: %s", data.c_str());

            // Парсим данные: PITCH:-10.00,YAW:20.00
            float pitch = 0.0f, yaw = 0.0f;
            if (sscanf(data.c_str(), "PITCH:%f,YAW:%f", &pitch, &yaw) == 2) {
                // ВЫВОД В ТЕРМИНАЛ
                RCLCPP_INFO(this->get_logger(), "📦 Получена команда: Pitch=%.2f, Yaw=%.2f", pitch, yaw);

                // Обработка команды
                process_gimbal_command(pitch, yaw);

                // Публикация в топик
                publish_received_angles(pitch, yaw);
            } else {
                RCLCPP_WARN(this->get_logger(), "Не удалось распарсить данные: %s", data.c_str());
            }
        } else if (bytes_read == 0) {
            RCLCPP_WARN(this->get_logger(), "Сервер закрыл соединение. Переподключение...");
            if (sock_ != -1) {
                close(sock_);
                sock_ = -1;
            }
            if (!reconnect_timer_ || !reconnect_timer_->is_canceled()) {
                start_reconnect_timer();
            }
        } else if (bytes_read == -1) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(this->get_logger(), "Ошибка чтения из сокета: %s (errno=%d)",
                             strerror(errno), errno);
                if (sock_ != -1) {
                    close(sock_);
                    sock_ = -1;
                }
                if (!reconnect_timer_ || !reconnect_timer_->is_canceled()) {
                    start_reconnect_timer();
                }
            }
        }
    }

    // ============================================
    // ОБРАБОТКА КОМАНД
    // ============================================
    void process_gimbal_command(float pitch, float yaw) {
        RCLCPP_DEBUG(this->get_logger(), "Обработка команды: Pitch=%.2f, Yaw=%.2f", pitch, yaw);
        last_pitch_ = pitch;
        last_yaw_ = yaw;

        // Здесь можно добавить дополнительную логику:
        // - Отправка на другую камеру
        // - Запись в файл
        // - Управление через сервис
    }

    // ============================================
    // ПЕРЕМЕННЫЕ
    // ============================================
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

    try {
        auto node = std::make_shared<TcpGimbalClient>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Критическая ошибка: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
