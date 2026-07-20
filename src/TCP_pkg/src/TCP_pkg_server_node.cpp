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
#include <regex>
#include <algorithm>
#include <termios.h>

// ============================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ (ЦВЕТА, ЛОГИ)
// ============================================
#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_BOLD    "\033[1m"

std::string get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) % 1000;
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S")
       << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

void log_info(const std::string& msg) {
    std::cout << COLOR_GREEN << "[" << get_timestamp() << "] [INFO] "
              << COLOR_RESET << msg << std::endl;
}

void log_warn(const std::string& msg) {
    std::cout << COLOR_YELLOW << "[" << get_timestamp() << "] [WARN] "
              << COLOR_RESET << msg << std::endl;
}

void log_error(const std::string& msg) {
    std::cout << COLOR_RED << "[" << get_timestamp() << "] [ERROR] "
              << COLOR_RESET << msg << std::endl;
}

void log_debug(const std::string& msg) {
    std::cout << COLOR_CYAN << "[" << get_timestamp() << "] [DEBUG] "
              << COLOR_RESET << msg << std::endl;
}

// ============================================
// ФУНКЦИЯ ДЛЯ ЧТЕНИЯ КЛАВИШ (НЕБЛОКИРУЮЩАЯ)
// ============================================
int getch_nonblock() {
    static struct termios oldt, newt;
    static bool initialized = false;

    if (!initialized) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        initialized = true;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    int ch = getchar();

    fcntl(STDIN_FILENO, F_SETFL, flags);

    if (ch == EOF) {
        return -1;
    }
    return ch;
}

class TcpServerNode : public rclcpp::Node {
public:
    TcpServerNode() : Node("tcp_server_node"), server_fd_(-1), running_(true), command_count_(0) {
        log_info("========================================");
        log_info("🌐 TCP СЕРВЕР ДЛЯ РОВЕРА");
        log_info("========================================");

        // ============================================
        // 1. ПАРАМЕТРЫ
        // ============================================
        this->declare_parameter("server_ip", "0.0.0.0");
        this->declare_parameter("server_port", 6000);
        this->declare_parameter("max_clients", 10);
        this->declare_parameter("gimbal_topic", "/gimbal/commands");
        this->declare_parameter("teleop_topic", "/teleop/data");
        this->declare_parameter("key_topic", "/teleop/key");
        this->declare_parameter("keyboard_commands_topic", "/keyboard_commands");
        this->declare_parameter("camera_cmd_topic", "/camera_control");
        this->declare_parameter("telega_cmd_topic", "/telega_commands");

        server_ip_ = this->get_parameter("server_ip").as_string();
        server_port_ = this->get_parameter("server_port").as_int();
        max_clients_ = this->get_parameter("max_clients").as_int();
        gimbal_topic_ = this->get_parameter("gimbal_topic").as_string();
        teleop_topic_ = this->get_parameter("teleop_topic").as_string();
        key_topic_ = this->get_parameter("key_topic").as_string();
        keyboard_commands_topic_ = this->get_parameter("keyboard_commands_topic").as_string();
        camera_cmd_topic_ = this->get_parameter("camera_cmd_topic").as_string();
        telega_cmd_topic_ = this->get_parameter("telega_cmd_topic").as_string();

        log_info("📋 ПАРАМЕТРЫ:");
        log_info("  server_ip: " + server_ip_);
        log_info("  server_port: " + std::to_string(server_port_));
        log_info("  max_clients: " + std::to_string(max_clients_));

        // ============================================
        // 2. ПОДПИСЧИКИ
        // ============================================
        gimbal_sub_ = this->create_subscription<mavros_msgs::msg::GimbalManagerSetPitchyaw>(
            gimbal_topic_, 10,
            std::bind(&TcpServerNode::gimbal_callback, this, std::placeholders::_1));

        teleop_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            teleop_topic_, 10,
            std::bind(&TcpServerNode::teleop_callback, this, std::placeholders::_1));

        key_sub_ = this->create_subscription<std_msgs::msg::String>(
            key_topic_, 10,
            std::bind(&TcpServerNode::key_callback, this, std::placeholders::_1));

        keyboard_commands_sub_ = this->create_subscription<std_msgs::msg::String>(
            keyboard_commands_topic_, 10,
            std::bind(&TcpServerNode::keyboard_commands_callback, this, std::placeholders::_1));

        // ============================================
        // 3. ПУБЛИКАТОРЫ
        // ============================================
        angle_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/server_received/angles", 10);

        camera_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
            camera_cmd_topic_, 10);

        telega_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
            telega_cmd_topic_, 10);

        // ============================================
        // 4. ЗАПУСК TCP-СЕРВЕРА
        // ============================================
        if (!start_server(server_ip_, server_port_)) {
            log_error("❌ Не удалось запустить сервер");
            return;
        }

        // ============================================
        // 5. ЗАПУСК ПОТОКОВ
        // ============================================
        read_thread_ = std::thread(&TcpServerNode::read_from_clients, this);
        keyboard_thread_ = std::thread(&TcpServerNode::read_keyboard, this);
        accept_thread_ = std::thread(&TcpServerNode::accept_clients, this);

        // ============================================
        // 6. ТАЙМЕРЫ
        // ============================================
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(15),
            std::bind(&TcpServerNode::print_stats, this));

        last_activity_time_ = std::chrono::steady_clock::now();

        log_info("========================================");
        log_info(COLOR_GREEN "✅ TCP-СЕРВЕР УСПЕШНО ЗАПУЩЕН" COLOR_RESET);
        log_info("  🌐 IP: " + server_ip_);
        log_info("  🔌 Порт: " + std::to_string(server_port_));
        log_info("========================================");
        log_info("");
        log_info("💡 Нажмите Ctrl+C для остановки сервера");
        log_info("");
    }

    ~TcpServerNode() {
        log_warn("🛑 ОСТАНОВКА СЕРВЕРА...");
        running_ = false;

        if (read_thread_.joinable()) read_thread_.join();
        if (keyboard_thread_.joinable()) keyboard_thread_.join();
        if (accept_thread_.joinable()) accept_thread_.join();

        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            for (int client : clients_) {
                if (client != -1) close(client);
            }
            clients_.clear();
        }

        if (server_fd_ != -1) {
            close(server_fd_);
        }

        log_info("✅ Сервер остановлен");
    }

private:
    // ============================================
    // ЗАПУСК TCP-СЕРВЕРА
    // ============================================
    bool start_server(const std::string& ip, int port) {
        log_info("📋 Создание сокета...");
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ == -1) {
            log_error("❌ Не удалось создать сокет: " + std::string(strerror(errno)));
            return false;
        }

        int opt = 1;
        setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in address;
        memset(&address, 0, sizeof(address));
        address.sin_family = AF_INET;
        address.sin_port = htons(port);

        if (inet_pton(AF_INET, ip.c_str(), &address.sin_addr) <= 0) {
            log_error("❌ Неверный IP-адрес: " + ip);
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
            log_error("❌ Ошибка привязки: " + std::string(strerror(errno)));
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        if (listen(server_fd_, max_clients_) < 0) {
            log_error("❌ Ошибка прослушивания: " + std::string(strerror(errno)));
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        int flags = fcntl(server_fd_, F_GETFL, 0);
        if (flags != -1) {
            fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK);
        }

        return true;
    }

    // ============================================
    // ПРИЁМ КЛИЕНТОВ
    // ============================================
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
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                if (clients_.size() >= static_cast<size_t>(max_clients_)) {
                    close(client_socket);
                    continue;
                }
                clients_.push_back(client_socket);
                log_info(COLOR_GREEN "✅ НОВЫЙ КЛИЕНТ (всего: " +
                         std::to_string(clients_.size()) + ")" COLOR_RESET);
            }
        }
    }

    // ============================================
    // ЧТЕНИЕ ОТ КЛИЕНТОВ
    // ============================================
    void read_from_clients() {
        char buffer[1024];
        while (running_ && rclcpp::ok()) {
            std::vector<int> clients_copy;
            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                clients_copy = clients_;
            }

            for (int client : clients_copy) {
                memset(buffer, 0, sizeof(buffer));
                ssize_t bytes_read = recv(client, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);

                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0';
                    std::string data(buffer);
                    std::istringstream stream(data);
                    std::string line;

                    while (std::getline(stream, line)) {
                        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
                        if (!line.empty()) {
                            process_client_command(line);
                        }
                    }
                }
                else if (bytes_read == 0) {
                    std::lock_guard<std::mutex> lock(clients_mutex_);
                    auto it = std::find(clients_.begin(), clients_.end(), client);
                    if (it != clients_.end()) {
                        close(*it);
                        clients_.erase(it);
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // ============================================
    // ОБРАБОТКА КОМАНД ОТ КЛИЕНТОВ
    // ============================================
    void process_client_command(const std::string& command) {
        auto msg = std_msgs::msg::String();
        msg.data = command;
        telega_cmd_pub_->publish(msg);
        command_count_++;
        last_activity_time_ = std::chrono::steady_clock::now();
        log_info("📨 От клиента: '" + command + "'");
    }

    // ============================================
    // ОТПРАВКА ВСЕМ КЛИЕНТАМ
    // ============================================
    void send_to_clients(const char* buffer, int len) {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        for (int client : clients_) {
            send(client, buffer, len, MSG_NOSIGNAL | MSG_DONTWAIT);
        }
    }

    // ============================================
    // КОЛБЭКИ
    // ============================================
    void gimbal_callback(const mavros_msgs::msg::GimbalManagerSetPitchyaw::SharedPtr msg) {
        command_count_++;
        last_activity_time_ = std::chrono::steady_clock::now();

        auto angle_msg = std_msgs::msg::Float32MultiArray();
        angle_msg.data.push_back(msg->pitch);
        angle_msg.data.push_back(msg->yaw);
        angle_publisher_->publish(angle_msg);

        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "CAMERA:%.2f,%.2f\n", msg->pitch, msg->yaw);
        send_to_clients(buffer, len);
    }

    void teleop_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;

        command_count_++;
        last_activity_time_ = std::chrono::steady_clock::now();

        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "TELEOP:%.2f,%.2f\n", msg->data[0], msg->data[1]);
        send_to_clients(buffer, len);
    }

    void key_callback(const std_msgs::msg::String::SharedPtr msg) {
        command_count_++;
        last_activity_time_ = std::chrono::steady_clock::now();
        std::string tcp_msg = "KEY:" + msg->data + "\n";
        send_to_clients(tcp_msg.c_str(), tcp_msg.length());
    }

    void keyboard_commands_callback(const std_msgs::msg::String::SharedPtr msg) {
        command_count_++;
        last_activity_time_ = std::chrono::steady_clock::now();
        std::string tcp_msg = "KEYBOARD:" + msg->data + "\n";
        send_to_clients(tcp_msg.c_str(), tcp_msg.length());
    }

    // ============================================
    // ЧТЕНИЕ КЛАВИАТУРЫ
    // ============================================
    void read_keyboard() {
        std::cout << COLOR_CYAN << "\n=== УПРАВЛЕНИЕ С КЛАВИАТУРЫ ===" << COLOR_RESET << std::endl;
        std::cout << "W/S - Вперед/Назад, A/D - Влево/Вправо" << std::endl;
        std::cout << "Q - Круиз, R/F - Скорость +/-" << std::endl;
        std::cout << "ПРОБЕЛ - Стоп" << std::endl;
        std::cout << "Стрелки - Камера" << std::endl;
        std::cout << "Z/X/C - Зум +/-/Стоп" << std::endl;

        while (running_ && rclcpp::ok()) {
            int ch = getch_nonblock();
            if (ch == -1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            std::string command;
            bool is_camera = false;

            switch (ch) {
            case 'w': case 'W': command = "w"; break;
            case 's': case 'S': command = "s"; break;
            case 'a': case 'A': command = "a"; break;
            case 'd': case 'D': command = "d"; break;
            case 'q': case 'Q': command = "q"; break;
            case 'r': case 'R': command = "r"; break;
            case 'f': case 'F': command = "f"; break;
            case ' ': command = " "; break;
            case 27: {
                int next = getch_nonblock();
                if (next == 91) {
                    int arrow = getch_nonblock();
                    switch (arrow) {
                    case 65: command = "UP"; is_camera = true; break;
                    case 66: command = "DOWN"; is_camera = true; break;
                    case 67: command = "RIGHT"; is_camera = true; break;
                    case 68: command = "LEFT"; is_camera = true; break;
                    }
                }
                break;
            }
            case 'z': case 'Z': command = "Z"; is_camera = true; break;
            case 'x': case 'X': command = "X"; is_camera = true; break;
            case 'c': case 'C': command = "C"; is_camera = true; break;
            default: break;
            }

            if (!command.empty()) {
                auto msg = std_msgs::msg::String();
                msg.data = command;

                if (is_camera) {
                    camera_cmd_pub_->publish(msg);
                    std::cout << "📷 Команда камере: " << command << std::endl;
                } else {
                    telega_cmd_pub_->publish(msg);
                    std::cout << "🚜 Команда тележке: " << command << std::endl;
                }

                command_count_++;
                last_activity_time_ = std::chrono::steady_clock::now();
            }
        }
    }

    // ============================================
    // СТАТИСТИКА
    // ============================================
    void print_stats() {
        std::cout << COLOR_MAGENTA << "\n========================================" << std::endl;
        std::cout << "📊 СТАТИСТИКА СЕРВЕРА" << std::endl;
        std::cout << "  👥 Клиентов: " << clients_.size() << " / " << max_clients_ << std::endl;
        std::cout << "  📝 Команд: " << command_count_ << std::endl;
        std::cout << "========================================\n" << COLOR_RESET << std::endl;
    }

    // ============================================
    // ПЕРЕМЕННЫЕ
    // ============================================
    std::string server_ip_;
    int server_port_;
    int max_clients_;
    std::string gimbal_topic_, teleop_topic_, key_topic_, keyboard_commands_topic_;
    std::string camera_cmd_topic_, telega_cmd_topic_;

    int server_fd_;
    std::vector<int> clients_;
    std::mutex clients_mutex_;
    std::thread accept_thread_, read_thread_, keyboard_thread_;
    std::atomic<bool> running_;

    rclcpp::Subscription<mavros_msgs::msg::GimbalManagerSetPitchyaw>::SharedPtr gimbal_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr teleop_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_commands_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr angle_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telega_cmd_pub_;

    rclcpp::TimerBase::SharedPtr stats_timer_;
    std::chrono::steady_clock::time_point last_activity_time_;
    int command_count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}