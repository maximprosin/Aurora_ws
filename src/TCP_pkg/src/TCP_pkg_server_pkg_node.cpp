#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/gimbal_manager_set_pitchyaw.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <vector>
#include <mutex>
#include <cstring>
#include <string>
#include <errno.h>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <map>
#include <chrono>

class TcpGimbalServer : public rclcpp::Node {
public:
    TcpGimbalServer() : Node("tcp_gimbal_server"), server_fd_(-1), running_(true) {
        // ============================================
        // 1. ПАРАМЕТРЫ
        // ============================================
        this->declare_parameter("server_ip", "0.0.0.0");
        this->declare_parameter("server_port", 5005);
        this->declare_parameter("max_clients", 10);
        this->declare_parameter("gimbal_topic", "/gimbal/commands");
        this->declare_parameter("teleop_topic", "/teleop/data");
        this->declare_parameter("key_topic", "/teleop/key");

        server_ip_ = this->get_parameter("server_ip").as_string();
        server_port_ = this->get_parameter("server_port").as_int();
        max_clients_ = this->get_parameter("max_clients").as_int();
        gimbal_topic_ = this->get_parameter("gimbal_topic").as_string();
        teleop_topic_ = this->get_parameter("teleop_topic").as_string();
        key_topic_ = this->get_parameter("key_topic").as_string();

        // ============================================
        // 2. ПОДПИСЧИКИ
        // ============================================
        // Камера (углы)
        gimbal_sub_ = this->create_subscription<mavros_msgs::msg::GimbalManagerSetPitchyaw>(
            gimbal_topic_, 10,
            std::bind(&TcpGimbalServer::gimbal_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "📋 Подписка на камеру: %s", gimbal_topic_.c_str());

        // Тележка (скорости)
        teleop_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            teleop_topic_, 10,
            std::bind(&TcpGimbalServer::teleop_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "📋 Подписка на тележку: %s", teleop_topic_.c_str());

        // Клавиши (новый подписчик!)
        key_sub_ = this->create_subscription<std_msgs::msg::String>(
            key_topic_, 10,
            std::bind(&TcpGimbalServer::key_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "📋 Подписка на клавиши: %s", key_topic_.c_str());

        // ============================================
        // 3. ПУБЛИКАТОР (для логов)
        // ============================================
        angle_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/server_received/angles", 10);

        // ============================================
        // 4. ИНИЦИАЛИЗАЦИЯ СТАТИСТИКИ
        // ============================================
        last_activity_time_ = this->now();
        command_count_ = 0;

        // ============================================
        // 5. ЗАПУСК TCP-СЕРВЕРА
        // ============================================
        if (!start_server(server_ip_, server_port_)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Не удалось запустить сервер");
            return;
        }

        // ============================================
        // 6. ТАЙМЕР ДЛЯ СТАТИСТИКИ
        // ============================================
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&TcpGimbalServer::print_stats, this));

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "🌐 TCP-сервер запущен на %s:%d", server_ip_.c_str(), server_port_);
        RCLCPP_INFO(this->get_logger(), "📋 Пересылает:");
        RCLCPP_INFO(this->get_logger(), "   - Камера: %s", gimbal_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "   - Тележка: %s", teleop_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "   - Клавиши: %s", key_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "========================================");
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
        RCLCPP_INFO(this->get_logger(), "🛑 Сервер остановлен. Всего команд: %d", command_count_);
    }

private:
    // ============================================
    // TCP-СЕРВЕР
    // ============================================
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

        int flags = fcntl(server_fd_, F_GETFL, 0);
        if (flags == -1 || fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK) == -1) {
            RCLCPP_WARN(this->get_logger(), "Не удалось установить неблокирующий режим");
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

                // Отправляем приветствие с управлением
                std::string welcome =
                    "Connected to ROS 2 Bridge\n"
                    "========================================\n"
                    "🚜 УПРАВЛЕНИЕ ТЕЛЕЖКОЙ:\n"
                    "  W - Вперёд    S - Назад\n"
                    "  A - Налево    D - Направо\n"
                    "  Q - Разворот налево    E - Разворот направо\n"
                    "  SPACE - Стоп\n"
                    "========================================\n"
                    "📷 УПРАВЛЕНИЕ КАМЕРОЙ:\n"
                    "  ↑ - Наклон вверх    ↓ - Наклон вниз\n"
                    "  ← - Поворот налево  → - Поворот направо\n"
                    "  + - Приближение     - - Отдаление\n"
                    "  Z - Зум приближение X - Зум отдаление\n"
                    "  C - Стоп зум\n"
                    "========================================\n"
                    "ℹ️  H - Справка\n"
                    "========================================\n\n";
                send(client_socket, welcome.c_str(), welcome.length(), MSG_NOSIGNAL | MSG_DONTWAIT);
            }
        }
    }

    // ============================================
    // КОЛБЭК №1: КАМЕРА
    // ============================================
    void gimbal_callback(const mavros_msgs::msg::GimbalManagerSetPitchyaw::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "📥 Камера: Pitch=%.2f, Yaw=%.2f", msg->pitch, msg->yaw);
        command_count_++;
        last_activity_time_ = this->now();

        auto angle_msg = std_msgs::msg::Float32MultiArray();
        angle_msg.data.clear();
        angle_msg.data.push_back(msg->pitch);
        angle_msg.data.push_back(msg->yaw);
        angle_publisher_->publish(angle_msg);

        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "CAMERA:%.2f,%.2f\n", msg->pitch, msg->yaw);
        send_to_clients(buffer, len);
    }

    // ============================================
    // КОЛБЭК №2: ТЕЛЕЖКА (скорости)
    // ============================================
    void teleop_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;

        float linear = msg->data[0];
        float angular = msg->data[1];

        RCLCPP_DEBUG(this->get_logger(), "📥 Тележка: linear=%.2f, angular=%.2f", linear, angular);
        command_count_++;
        last_activity_time_ = this->now();

        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "TELEOP:%.2f,%.2f\n", linear, angular);
        send_to_clients(buffer, len);
    }

    // ============================================
    // КОЛБЭК №3: КЛАВИШИ (НОВЫЙ!)
    // ============================================
    void key_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string key = msg->data;

        // Маппинг клавиш на понятные названия
        std::map<std::string, std::string> key_names = {
            {"W", "Вперёд"},
            {"S", "Назад"},
            {"A", "Налево"},
            {"D", "Направо"},
            {"Q", "Разворот налево"},
            {"E", "Разворот направо"},
            {"SPACE", "СТОП"},
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
        auto it = key_names.find(key);
        if (it != key_names.end()) {
            display_name = it->second;
        }

        RCLCPP_INFO(this->get_logger(), "⌨️ Клавиша: [%s] -> %s", key.c_str(), display_name.c_str());
        command_count_++;
        last_activity_time_ = this->now();

        // Отправляем в TCP
        char buffer[256];
        int len = snprintf(buffer, sizeof(buffer), "KEY:%s\n", key.c_str());
        send_to_clients(buffer, len);
    }

    // ============================================
    // ОТПРАВКА ВСЕМ КЛИЕНТАМ
    // ============================================
    void send_to_clients(const char* buffer, int len) {
        std::lock_guard<std::mutex> lock(clients_mutex_);

        if (clients_.empty()) return;

        for (auto it = clients_.begin(); it != clients_.end();) {
            int client = *it;
            ssize_t bytes_sent = send(client, buffer, len, MSG_NOSIGNAL | MSG_DONTWAIT);

            if (bytes_sent < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    RCLCPP_WARN(this->get_logger(), "Клиент отключился (send: %s). Удаляем.", strerror(errno));
                    close(client);
                    it = clients_.erase(it);
                } else {
                    ++it;
                }
            } else {
                RCLCPP_DEBUG(this->get_logger(), "✅ Отправлено %zd байт: %s", bytes_sent, buffer);
                ++it;
            }
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
        RCLCPP_INFO(this->get_logger(), "   Подключено клиентов: %zu", clients_.size());
        RCLCPP_INFO(this->get_logger(), "   Активность: %.0f сек назад", dt);
    }

    // ============================================
    // ПЕРЕМЕННЫЕ
    // ============================================
    std::string server_ip_;
    int server_port_;
    int max_clients_;
    std::string gimbal_topic_;
    std::string teleop_topic_;
    std::string key_topic_;

    int server_fd_;
    std::vector<int> clients_;
    std::mutex clients_mutex_;
    std::thread accept_thread_;
    std::atomic<bool> running_;

    // Три подписчика!
    rclcpp::Subscription<mavros_msgs::msg::GimbalManagerSetPitchyaw>::SharedPtr gimbal_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr teleop_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr angle_publisher_;

    // Статистика
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::Time last_activity_time_;
    int command_count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpGimbalServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
