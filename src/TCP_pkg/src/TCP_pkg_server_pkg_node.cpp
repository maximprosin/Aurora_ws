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

        // ============================================
        // 2. ПОДПИСЧИКИ (получение данных из ROS 2)
        // ============================================
        gimbal_sub_ = this->create_subscription<mavros_msgs::msg::GimbalManagerSetPitchyaw>(
            gimbal_topic_, 10,
            std::bind(&TcpGimbalServer::gimbal_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "📋 Подписка на камеру: %s", gimbal_topic_.c_str());

        teleop_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            teleop_topic_, 10,
            std::bind(&TcpGimbalServer::teleop_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "📋 Подписка на тележку: %s", teleop_topic_.c_str());

        key_sub_ = this->create_subscription<std_msgs::msg::String>(
            key_topic_, 10,
            std::bind(&TcpGimbalServer::key_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "📋 Подписка на клавиши: %s", key_topic_.c_str());

        keyboard_commands_sub_ = this->create_subscription<std_msgs::msg::String>(
            keyboard_commands_topic_, 10,
            std::bind(&TcpGimbalServer::keyboard_commands_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "📋 Подписка на команды клавиатуры: %s", keyboard_commands_topic_.c_str());

        // ============================================
        // 3. ПУБЛИКАТОРЫ (отправка команд в ROS 2)
        // ============================================
        angle_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/server_received/angles", 10);

        camera_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
            camera_cmd_topic_, 10);
        RCLCPP_INFO(this->get_logger(), "📤 Публикация команд камеры: %s", camera_cmd_topic_.c_str());

        telega_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
            telega_cmd_topic_, 10);
        RCLCPP_INFO(this->get_logger(), "📤 Публикация команд тележки: %s", telega_cmd_topic_.c_str());

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
        // 6. ЗАПУСК ПОТОКОВ
        // ============================================
        read_thread_ = std::thread(&TcpGimbalServer::read_from_clients, this);
        keyboard_thread_ = std::thread(&TcpGimbalServer::read_keyboard, this);  // НОВЫЙ ПОТОК ДЛЯ КЛАВИАТУРЫ

        // ============================================
        // 7. ТАЙМЕР ДЛЯ СТАТИСТИКИ
        // ============================================
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&TcpGimbalServer::print_stats, this));

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "🌐 TCP-сервер запущен на %s:%d", server_ip_.c_str(), server_port_);
        RCLCPP_INFO(this->get_logger(), "⌨️  Управление с клавиатуры сервера");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

    ~TcpGimbalServer() {
        running_ = false;

        if (read_thread_.joinable()) {
            read_thread_.join();
        }

        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }

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

        // Восстанавливаем терминал
        tcgetattr(STDIN_FILENO, &old_termios_);

        RCLCPP_INFO(this->get_logger(), "🛑 Сервер остановлен. Всего команд: %d", command_count_);
    }

private:
    // ============================================
    // НОВОЕ! ЧТЕНИЕ КЛАВИАТУРЫ НА СЕРВЕРЕ
    // ============================================
    void read_keyboard() {
        std::cout << "\n========================================" << std::endl;
        std::cout << "УПРАВЛЕНИЕ С КЛАВИАТУРЫ СЕРВЕРА:" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "🚜 ТЕЛЕЖКА:" << std::endl;
        std::cout << "  W/S - Вперёд/Назад" << std::endl;
        std::cout << "  A/D - Налево/Направо" << std::endl;
        std::cout << "  Q   - Круиз-контроль" << std::endl;
        std::cout << "  R/F - Увеличить/Уменьшить скорость" << std::endl;
        std::cout << "  ПРОБЕЛ - Стоп" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "📷 КАМЕРА:" << std::endl;
        std::cout << "  СТРЕЛКИ - Наклон/Поворот" << std::endl;
        std::cout << "  +/- - Приближение/Отдаление" << std::endl;
        std::cout << "  Z/X/C - Зум +/-/Стоп" << std::endl;
        std::cout << "========================================\n" << std::endl;

        while (running_ && rclcpp::ok()) {
            int ch = getch_nonblock();

            if (ch == -1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            std::string ros_command;
            std::string tcp_command;
            bool send_to_tcp = true;

            // Обработка клавиш
            switch (ch) {
            // ТЕЛЕЖКА
            case 'w': case 'W':
                ros_command = "w";
                tcp_command = "KEY:W:press\n";
                std::cout << "🚜 Вперёд" << std::endl;
                telega_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 's': case 'S':
                ros_command = "s";
                tcp_command = "KEY:S:press\n";
                std::cout << "🚜 Назад" << std::endl;
                telega_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 'a': case 'A':
                ros_command = "a";
                tcp_command = "KEY:A:press\n";
                std::cout << "🚜 Налево" << std::endl;
                telega_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 'd': case 'D':
                ros_command = "d";
                tcp_command = "KEY:D:press\n";
                std::cout << "🚜 Направо" << std::endl;
                telega_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 'q': case 'Q':
                ros_command = "q";
                tcp_command = "KEY:Q:press\n";
                std::cout << "🚜 Круиз-контроль" << std::endl;
                telega_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 'r': case 'R':
                ros_command = "r";
                tcp_command = "KEY:R:press\n";
                std::cout << "🚜 Скорость +" << std::endl;
                telega_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 'f': case 'F':
                ros_command = "f";
                tcp_command = "KEY:F:press\n";
                std::cout << "🚜 Скорость -" << std::endl;
                telega_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case ' ':
                ros_command = " ";
                tcp_command = "KEY:SPACE:press\n";
                std::cout << "🚜 СТОП" << std::endl;
                telega_cmd_pub_->publish(create_string_msg(ros_command));
                break;

            // КАМЕРА (стрелки)
            case 27: // ESC последовательность
            {
                int next = getch_nonblock();
                if (next == 91) { // '['
                    int arrow = getch_nonblock();
                    switch (arrow) {
                    case 65: // Вверх
                        ros_command = "UP";
                        tcp_command = "KEY:UP:press\n";
                        std::cout << "📷 Наклон вверх" << std::endl;
                        camera_cmd_pub_->publish(create_string_msg(ros_command));
                        break;
                    case 66: // Вниз
                        ros_command = "DOWN";
                        tcp_command = "KEY:DOWN:press\n";
                        std::cout << "📷 Наклон вниз" << std::endl;
                        camera_cmd_pub_->publish(create_string_msg(ros_command));
                        break;
                    case 68: // Влево
                        ros_command = "LEFT";
                        tcp_command = "KEY:LEFT:press\n";
                        std::cout << "📷 Поворот налево" << std::endl;
                        camera_cmd_pub_->publish(create_string_msg(ros_command));
                        break;
                    case 67: // Вправо
                        ros_command = "RIGHT";
                        tcp_command = "KEY:RIGHT:press\n";
                        std::cout << "📷 Поворот направо" << std::endl;
                        camera_cmd_pub_->publish(create_string_msg(ros_command));
                        break;
                    }
                }
            }
            break;

            // КАМЕРА: + и -
            case '+': case '=':
                ros_command = "PLUS";
                tcp_command = "KEY:PLUS:press\n";
                std::cout << "📷 Приближение" << std::endl;
                camera_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case '-': case '_':
                ros_command = "MINUS";
                tcp_command = "KEY:MINUS:press\n";
                std::cout << "📷 Отдаление" << std::endl;
                camera_cmd_pub_->publish(create_string_msg(ros_command));
                break;

            // КАМЕРА: Z, X, C
            case 'z': case 'Z':
                ros_command = "Z";
                tcp_command = "KEY:Z:press\n";
                std::cout << "📷 Зум +" << std::endl;
                camera_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 'x': case 'X':
                ros_command = "X";
                tcp_command = "KEY:X:press\n";
                std::cout << "📷 Зум -" << std::endl;
                camera_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 'c': case 'C':
                ros_command = "C";
                tcp_command = "KEY:C:press\n";
                std::cout << "📷 Стоп зум" << std::endl;
                camera_cmd_pub_->publish(create_string_msg(ros_command));
                break;

            default:
                send_to_tcp = false;
                break;
            }

            // Отправляем TCP клиентам
            if (send_to_tcp && !tcp_command.empty()) {
                send_to_clients(tcp_command.c_str(), tcp_command.length());
            }

            command_count_++;
            last_activity_time_ = this->now();
        }
    }

    // Вспомогательная функция для создания String сообщения
    std_msgs::msg::String create_string_msg(const std::string& data) {
        auto msg = std_msgs::msg::String();
        msg.data = data;
        return msg;
    }

    // ============================================
    // TCP-СЕРВЕР: ЗАПУСК
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

    // ============================================
    // TCP-СЕРВЕР: ПРИЁМ КЛИЕНТОВ
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
            }
        }
    }

    // ============================================
    // ЧТЕНИЕ КОМАНД ОТ КЛИЕНТОВ
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
                            RCLCPP_INFO(this->get_logger(), "📨 От клиента: '%s'", line.c_str());
                            process_client_command(line);
                        }
                    }
                }
                else if (bytes_read == 0) {
                    RCLCPP_WARN(this->get_logger(), "Клиент отключился (recv=0). Удаляем.");
                    std::lock_guard<std::mutex> lock(clients_mutex_);
                    auto it = std::find(clients_.begin(), clients_.end(), client);
                    if (it != clients_.end()) {
                        close(*it);
                        clients_.erase(it);
                    }
                }
                else {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        RCLCPP_WARN(this->get_logger(), "Ошибка recv(): %s. Удаляем клиента.", strerror(errno));
                        std::lock_guard<std::mutex> lock(clients_mutex_);
                        auto it = std::find(clients_.begin(), clients_.end(), client);
                        if (it != clients_.end()) {
                            close(*it);
                            clients_.erase(it);
                        }
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

        if (command.find("TELEGA:") == 0) {
            std::string telega_cmd = command.substr(7);
            std::map<std::string, std::string> telega_mapping = {
                {"W", "w"}, {"S", "s"}, {"A", "a"}, {"D", "d"},
                {"Q", "q"}, {"R", "r"}, {"F", "f"},
                {"UP", "w"}, {"DOWN", "s"}, {"LEFT", "a"}, {"RIGHT", "d"},
                {"SPACE", " "}, {"STOP", "stop"}
            };

            auto it = telega_mapping.find(telega_cmd);
            msg.data = (it != telega_mapping.end()) ? it->second : telega_cmd;
            telega_cmd_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "🚜 Команда тележке: '%s' -> '%s'", telega_cmd.c_str(), msg.data.c_str());
        }
        else if (command.find("CAMERA:") == 0) {
            std::string camera_cmd = command.substr(7);
            std::map<std::string, std::string> camera_mapping = {
                {"UP", "UP"}, {"DOWN", "DOWN"}, {"LEFT", "LEFT"}, {"RIGHT", "RIGHT"},
                {"PLUS", "PLUS"}, {"MINUS", "MINUS"},
                {"Z", "Z"}, {"X", "X"}, {"C", "C"}, {"STOP", "STOP"},
                {"ZOOM_IN", "Z"}, {"ZOOM_OUT", "X"}, {"ZOOM_STOP", "C"}
            };

            auto it = camera_mapping.find(camera_cmd);
            msg.data = (it != camera_mapping.end()) ? it->second : camera_cmd;
            camera_cmd_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "📷 Команда камере: '%s' -> '%s'", camera_cmd.c_str(), msg.data.c_str());
        }
        else {
            std::map<std::string, std::string> direct_mapping = {
                {"w", "w"}, {"W", "w"}, {"s", "s"}, {"S", "s"},
                {"a", "a"}, {"A", "a"}, {"d", "d"}, {"D", "d"},
                {"q", "q"}, {"Q", "q"}, {"r", "r"}, {"R", "r"},
                {"f", "f"}, {"F", "f"}, {" ", " "}, {"SPACE", " "},
                {"UP", "UP"}, {"DOWN", "DOWN"}, {"LEFT", "LEFT"}, {"RIGHT", "RIGHT"},
                {"PLUS", "PLUS"}, {"MINUS", "MINUS"},
                {"Z", "Z"}, {"X", "X"}, {"C", "C"}, {"STOP", "STOP"}
            };

            auto it = direct_mapping.find(command);
            if (it != direct_mapping.end()) {
                std::string mapped_cmd = it->second;

                if (mapped_cmd == "UP" || mapped_cmd == "DOWN" ||
                    mapped_cmd == "LEFT" || mapped_cmd == "RIGHT" ||
                    mapped_cmd == "PLUS" || mapped_cmd == "MINUS" ||
                    mapped_cmd == "Z" || mapped_cmd == "X" || mapped_cmd == "C" ||
                    mapped_cmd == "STOP") {
                    msg.data = mapped_cmd;
                    camera_cmd_pub_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "📷 Прямая команда камере: '%s'", mapped_cmd.c_str());
                } else {
                    msg.data = mapped_cmd;
                    telega_cmd_pub_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "🚜 Прямая команда тележке: '%s'", mapped_cmd.c_str());
                }
            } else {
                msg.data = command;
                telega_cmd_pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "🔄 Неизвестная команда тележке: '%s'", command.c_str());
            }
        }

        command_count_++;
        last_activity_time_ = this->now();
    }

    // ============================================
    // КОЛБЭКИ (КАМЕРА, ТЕЛЕЖКА, КЛАВИШИ)
    // ============================================
    void gimbal_callback(const mavros_msgs::msg::GimbalManagerSetPitchyaw::SharedPtr msg) {
        command_count_++;
        last_activity_time_ = this->now();

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

        float linear = msg->data[0];
        float angular = msg->data[1];
        command_count_++;
        last_activity_time_ = this->now();

        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "TELEOP:%.2f,%.2f\n", linear, angular);
        send_to_clients(buffer, len);
    }

    void key_callback(const std_msgs::msg::String::SharedPtr msg) {
        command_count_++;
        last_activity_time_ = this->now();
        char buffer[256];
        int len = snprintf(buffer, sizeof(buffer), "KEY:%s\n", msg->data.c_str());
        send_to_clients(buffer, len);
    }

    void keyboard_commands_callback(const std_msgs::msg::String::SharedPtr msg) {
        command_count_++;
        last_activity_time_ = this->now();
        std::string tcp_message = "KEYBOARD:" + msg->data + "\n";
        send_to_clients(tcp_message.c_str(), tcp_message.length());
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
                    close(client);
                    it = clients_.erase(it);
                } else {
                    ++it;
                }
            } else {
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
        RCLCPP_INFO(this->get_logger(), "📊 Клиентов: %zu | Команд: %d | Активность: %.0f сек назад",
                    clients_.size(), command_count_, dt);
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
    struct termios old_termios_;

    // Подписчики
    rclcpp::Subscription<mavros_msgs::msg::GimbalManagerSetPitchyaw>::SharedPtr gimbal_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr teleop_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_commands_sub_;

    // Публикаторы
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr angle_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telega_cmd_pub_;

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
