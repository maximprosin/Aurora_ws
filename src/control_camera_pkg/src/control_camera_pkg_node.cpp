#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/gimbal_manager_pitchyaw.hpp>
#include <mavros_msgs/msg/gimbal_manager_set_pitchyaw.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <algorithm>
#include <atomic>
#include <csignal>
#include <sstream>
#include <iomanip>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>

class KeyboardGimbalController : public rclcpp::Node {
public:
    KeyboardGimbalController() : Node("keyboard_gimbal_controller"), running_(true) {
        // ============================================
        // 1. ПАРАМЕТРИЗАЦИЯ
        // ============================================
        this->declare_parameter("publish_topic", "/gimbal/commands");
        this->declare_parameter("angle_publish_topic", "/gimbal/angles");
        this->declare_parameter("service_name", "/mavros/gimbal_control/manager/pitchyaw");
        this->declare_parameter("gimbal_device_id", 1);
        this->declare_parameter("flags", 0);
        this->declare_parameter("step_angle", 2.0f);
        this->declare_parameter("max_pitch", 25.0f);
        this->declare_parameter("min_pitch", -90.0f);
        this->declare_parameter("max_yaw", 180.0f);
        this->declare_parameter("min_yaw", -180.0f);
        this->declare_parameter("keyboard_poll_rate_hz", 50);
        this->declare_parameter("service_timeout_sec", 5.0);
        this->declare_parameter("zoom_speed", 0.2f);

        // Параметры для UDP-команд SiYi
        this->declare_parameter("camera_ip", "192.168.144.25");
        this->declare_parameter("camera_port", 14550);
        this->declare_parameter("vdisp_rgb", 3);          // Single Zoom
        this->declare_parameter("vdisp_wide_thermal", 5); // Wide или Thermal

        // Загрузка параметров
        publish_topic_ = this->get_parameter("publish_topic").as_string();
        angle_publish_topic_ = this->get_parameter("angle_publish_topic").as_string();
        service_name_ = this->get_parameter("service_name").as_string();
        gimbal_device_id_ = this->get_parameter("gimbal_device_id").as_int();
        flags_ = this->get_parameter("flags").as_int();
        step_ = this->get_parameter("step_angle").as_double();
        max_pitch_ = this->get_parameter("max_pitch").as_double();
        min_pitch_ = this->get_parameter("min_pitch").as_double();
        max_yaw_ = this->get_parameter("max_yaw").as_double();
        min_yaw_ = this->get_parameter("min_yaw").as_double();
        keyboard_poll_rate_hz_ = this->get_parameter("keyboard_poll_rate_hz").as_int();
        service_timeout_sec_ = this->get_parameter("service_timeout_sec").as_double();
        zoom_speed_ = this->get_parameter("zoom_speed").as_double();

        camera_ip_ = this->get_parameter("camera_ip").as_string();
        camera_port_ = this->get_parameter("camera_port").as_int();
        vdisp_rgb_ = this->get_parameter("vdisp_rgb").as_int();
        vdisp_wide_thermal_ = this->get_parameter("vdisp_wide_thermal").as_int();

        // ============================================
        // 2. ПУБЛИКАТОРЫ
        // ============================================
        publisher_ = this->create_publisher<mavros_msgs::msg::GimbalManagerSetPitchyaw>(
            publish_topic_, 10);
        angle_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            angle_publish_topic_, 10);
        stream_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/set_rtsp_stream", 10);

        // ============================================
        // 3. КЛИЕНТЫ MAVROS
        // ============================================
        client_ = this->create_client<mavros_msgs::srv::GimbalManagerPitchyaw>(service_name_);
        if (!wait_for_service_with_timeout()) {
            RCLCPP_ERROR(this->get_logger(),
                         "❌ Сервис %s не доступен после %.1f сек.",
                         service_name_.c_str(), service_timeout_sec_);
        } else {
            RCLCPP_INFO(this->get_logger(), "✅ Сервис %s доступен", service_name_.c_str());
        }

        cmd_client_ = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
        if (!cmd_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Сервис /mavros/cmd/command не доступен");
        } else {
            RCLCPP_INFO(this->get_logger(), "✅ Сервис /mavros/cmd/command доступен");
        }

        // ============================================
        // 4. ТЕРМИНАЛ
        // ============================================
        if (!setup_terminal()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Не удалось настроить терминал.");
        }

        // ============================================
        // 5. ТАЙМЕР
        // ============================================
        int poll_interval_ms = 1000 / keyboard_poll_rate_hz_;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(poll_interval_ms),
            std::bind(&KeyboardGimbalController::read_keyboard, this));

        // ============================================
        // 6. ВЫВОД
        // ============================================
        RCLCPP_INFO(this->get_logger(), "=== Узел управления камерой запущен ===");
        RCLCPP_INFO(this->get_logger(), "📊 Текущие параметры:");
        RCLCPP_INFO(this->get_logger(), "  Шаг: %.1f°", step_);
        RCLCPP_INFO(this->get_logger(), "  Скорость зума: %.2f", zoom_speed_);
        RCLCPP_INFO(this->get_logger(), "  Наклон (pitch): [%.1f° .. %.1f°]", min_pitch_, max_pitch_);
        RCLCPP_INFO(this->get_logger(), "  Поворот (yaw): [%.1f° .. %.1f°]", min_yaw_, max_yaw_);
        RCLCPP_INFO(this->get_logger(), "  Камера IP: %s, порт: %d", camera_ip_.c_str(), camera_port_);
        RCLCPP_INFO(this->get_logger(), "  vdisp RGB (video2): %d", vdisp_rgb_);
        RCLCPP_INFO(this->get_logger(), "  vdisp Wide/Thermal (video1): %d", vdisp_wide_thermal_);
        print_help();
    }

    ~KeyboardGimbalController() {
        running_ = false;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_) == 0) {
            RCLCPP_DEBUG(this->get_logger(), "Настройки терминала восстановлены");
        }
    }

private:
    // --- Справка ---
    void print_help() {
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "📋 Управление подвесом (стрелки):");
        RCLCPP_INFO(this->get_logger(), "  ↑ - наклон вверх");
        RCLCPP_INFO(this->get_logger(), "  ↓ - наклон вниз");
        RCLCPP_INFO(this->get_logger(), "  ← - поворот влево");
        RCLCPP_INFO(this->get_logger(), "  → - поворот вправо");
        RCLCPP_INFO(this->get_logger(), "  ПРОБЕЛ - сброс в центр (0°, 0°)");
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "📋 Управление зумом:");
        RCLCPP_INFO(this->get_logger(), "  Z - приближение");
        RCLCPP_INFO(this->get_logger(), "  X - отдаление");
        RCLCPP_INFO(this->get_logger(), "  C - остановить зум");
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "📋 Переключение видеопотоков:");
        RCLCPP_INFO(this->get_logger(), "  1 - Основной поток video2 (RGB ЗУМ)");
        RCLCPP_INFO(this->get_logger(), "  2 - Дополнительный поток video1 (WIDE или THERMAL)");
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "📋 Настройки:");
        RCLCPP_INFO(this->get_logger(), "  + / - - изменить шаг угла");
        RCLCPP_INFO(this->get_logger(), "  H/h - справка");
        RCLCPP_INFO(this->get_logger(), "  ESC - выход");
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "📊 Текущий шаг: %.1f°", step_);
        RCLCPP_INFO(this->get_logger(), "📊 Скорость зума: %.2f", zoom_speed_);
    }

    // --- Вспомогательные методы ---
    bool setup_terminal() {
        if (tcgetattr(STDIN_FILENO, &old_termios_) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Не удалось получить настройки терминала: %s", strerror(errno));
            return false;
        }
        struct termios new_termios = old_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        new_termios.c_cc[VMIN] = 0;
        new_termios.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &new_termios) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Не удалось установить настройки терминала: %s", strerror(errno));
            return false;
        }
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags == -1) {
            RCLCPP_ERROR(this->get_logger(), "Не удалось получить флаги STDIN: %s", strerror(errno));
            return false;
        }
        if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Не удалось сделать STDIN неблокирующим: %s", strerror(errno));
            return false;
        }
        return true;
    }

    bool wait_for_service_with_timeout() {
        auto start_time = std::chrono::steady_clock::now();
        auto timeout = std::chrono::duration<double>(service_timeout_sec_);
        while (rclcpp::ok() && running_) {
            if (client_->wait_for_service(std::chrono::milliseconds(100))) {
                return true;
            }
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed > timeout) {
                return false;
            }
        }
        return false;
    }

    int get_key() {
        char ch;
        ssize_t bytes_read = read(STDIN_FILENO, &ch, 1);
        if (bytes_read == 1) {
            return static_cast<int>(ch);
        }
        return -1;
    }

    // --- Публикация углов ---
    void publish_angles(float pitch, float yaw) {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data.clear();
        msg.data.push_back(static_cast<float>(pitch));
        msg.data.push_back(static_cast<float>(yaw));
        angle_publisher_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "📤 Опубликованы углы: Pitch=%.1f, Yaw=%.1f", pitch, yaw);
    }

    // --- Отправка команды подвесу ---
    void send_gimbal_command(float pitch, float yaw) {
        if (pitch < min_pitch_ || pitch > max_pitch_) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Наклон %.1f° выходит за пределы [%.1f°..%.1f°]",
                        pitch, min_pitch_, max_pitch_);
            return;
        }
        if (yaw < min_yaw_ || yaw > max_yaw_) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Поворот %.1f° выходит за пределы [%.1f°..%.1f°]",
                        yaw, min_yaw_, max_yaw_);
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::GimbalManagerPitchyaw::Request>();
        request->pitch = pitch;
        request->yaw = yaw;
        request->pitch_rate = 0.0f;
        request->yaw_rate = 0.0f;
        request->flags = flags_;
        request->gimbal_device_id = gimbal_device_id_;

        if (client_->service_is_ready()) {
            client_->async_send_request(request,
                                        [this](rclcpp::Client<mavros_msgs::srv::GimbalManagerPitchyaw>::SharedFuture future) {
                                            auto result = future.get();
                                            if (result->result != 0) {
                                                RCLCPP_WARN(this->get_logger(),
                                                            "❌ Команда подвеса не выполнена: result=%d", result->result);
                                            } else {
                                                RCLCPP_DEBUG(this->get_logger(), "✅ Команда подвеса выполнена");
                                            }
                                        });
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ Сервис MAVROS не доступен.");
        }

        auto msg = mavros_msgs::msg::GimbalManagerSetPitchyaw();
        msg.pitch = pitch;
        msg.yaw = yaw;
        msg.pitch_rate = 0.0f;
        msg.yaw_rate = 0.0f;
        msg.flags = flags_;
        msg.gimbal_device_id = gimbal_device_id_;
        publisher_->publish(msg);
        publish_angles(pitch, yaw);
    }

    // ============================================================
    // ★ УПРАВЛЕНИЕ ЗУМОМ (MAVLink) ★
    // ============================================================
    void send_zoom_continuous(float speed) {
        if (!cmd_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Сервис /mavros/cmd/command не готов");
            return;
        }
        auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        req->command = 531;
        req->param1 = 1.0f;
        req->param2 = std::clamp(speed, -1.0f, 1.0f);
        req->confirmation = 0;
        cmd_client_->async_send_request(req,
                                        [this](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
                                            auto result = future.get();
                                            if (result->success) {
                                                RCLCPP_DEBUG(this->get_logger(), "✅ Команда зума выполнена успешно");
                                            } else {
                                                RCLCPP_WARN(this->get_logger(), "❌ Команда зума не выполнена");
                                            }
                                        });
        RCLCPP_INFO(this->get_logger(), "🔍 Зум: скорость %.2f", req->param2);
    }

    void send_zoom_stop() {
        if (!cmd_client_->service_is_ready()) return;
        auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        req->command = 531;
        req->param1 = 1.0f;
        req->param2 = 0.0f;
        req->confirmation = 0;
        cmd_client_->async_send_request(req,
                                        [this](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
                                            auto result = future.get();
                                            if (result->success) {
                                                RCLCPP_DEBUG(this->get_logger(), "✅ Остановка зума выполнена");
                                            } else {
                                                RCLCPP_WARN(this->get_logger(), "❌ Остановка зума не выполнена");
                                            }
                                        });
        RCLCPP_INFO(this->get_logger(), "⏹️ Остановка зума (положение зафиксировано)");
    }

    // ============================================================
    // ★ ПРЯМАЯ UDP-КОМАНДА SiYi (CMD_ID:0x11) ★
    // ============================================================
    void send_siyi_command(uint8_t cmd_id, uint8_t data) {
        int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Не удалось создать UDP-сокет");
            return;
        }

        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(camera_port_);
        if (inet_pton(AF_INET, camera_ip_.c_str(), &server_addr.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Неверный IP-адрес: %s", camera_ip_.c_str());
            close(sockfd);
            return;
        }

        uint8_t packet[4] = {0x55, 0x01, cmd_id, data};
        ssize_t sent = sendto(sockfd, packet, sizeof(packet), 0,
                              (struct sockaddr*)&server_addr, sizeof(server_addr));
        close(sockfd);

        if (sent == sizeof(packet)) {
            RCLCPP_INFO(this->get_logger(), "✅ UDP-команда 0x%02X с data=0x%02X отправлена на %s:%d",
                        cmd_id, data, camera_ip_.c_str(), camera_port_);
        } else {
            RCLCPP_WARN(this->get_logger(), "❌ Не удалось отправить UDP-команду");
        }
    }

    // --- Переключение режима изображения через UDP ---
    void set_image_mode(int vdisp_mode, const std::string& mode_name) {
        RCLCPP_INFO(this->get_logger(), "📷 Переключение в режим %s (vdisp=%d)", mode_name.c_str(), vdisp_mode);
        send_siyi_command(0x11, static_cast<uint8_t>(vdisp_mode));
    }

    // --- Обработка клавиш ---
    void read_keyboard() {
        int key = get_key();
        if (key == -1) return;

        // Обработка стрелок
        if (key == 27) {
            if (get_key() == 91) {
                int arrow_key = get_key();
                float new_pitch = pitch_;
                float new_yaw = yaw_;
                bool command_sent = false;

                switch (arrow_key) {
                case 65: // ↑
                    new_pitch = pitch_ + step_;
                    if (new_pitch > max_pitch_) {
                        new_pitch = max_pitch_;
                        RCLCPP_INFO(this->get_logger(), "🔒 Максимум наклона: %.1f°", new_pitch);
                    } else {
                        RCLCPP_INFO(this->get_logger(), "📐 Наклон вверх: %.1f°", new_pitch);
                    }
                    command_sent = true;
                    break;
                case 66: // ↓
                    new_pitch = pitch_ - step_;
                    if (new_pitch < min_pitch_) {
                        new_pitch = min_pitch_;
                        RCLCPP_INFO(this->get_logger(), "🔒 Минимум наклона: %.1f°", new_pitch);
                    } else {
                        RCLCPP_INFO(this->get_logger(), "📐 Наклон вниз: %.1f°", new_pitch);
                    }
                    command_sent = true;
                    break;
                case 67: // →
                    new_yaw = yaw_ - step_;
                    if (new_yaw < min_yaw_) {
                        new_yaw = min_yaw_;
                        RCLCPP_INFO(this->get_logger(), "🔒 Минимум поворота: %.1f°", new_yaw);
                    } else {
                        RCLCPP_INFO(this->get_logger(), "📐 Поворот вправо: %.1f°", new_yaw);
                    }
                    command_sent = true;
                    break;
                case 68: // ←
                    new_yaw = yaw_ + step_;
                    if (new_yaw > max_yaw_) {
                        new_yaw = max_yaw_;
                        RCLCPP_INFO(this->get_logger(), "🔒 Максимум поворота: %.1f°", new_yaw);
                    } else {
                        RCLCPP_INFO(this->get_logger(), "📐 Поворот влево: %.1f°", new_yaw);
                    }
                    command_sent = true;
                    break;
                default:
                    break;
                }

                if (command_sent) {
                    pitch_ = new_pitch;
                    yaw_ = new_yaw;
                    send_gimbal_command(pitch_, yaw_);
                }
                return;
            }
            // ESC
            RCLCPP_INFO(this->get_logger(), "⏹️ Выход...");
            running_ = false;
            rclcpp::shutdown();
            return;
        }

        float new_pitch = pitch_;
        float new_yaw = yaw_;
        bool command_sent = false;

        switch (key) {
        case ' ':
            new_pitch = 0.0f;
            new_yaw = 0.0f;
            RCLCPP_INFO(this->get_logger(), "🔄 Сброс в центр (0°, 0°)");
            command_sent = true;
            break;

        // Зум
        case 'Z': case 'z':
            send_zoom_continuous(zoom_speed_);
            break;
        case 'X': case 'x':
            send_zoom_continuous(-zoom_speed_);
            break;
        case 'C': case 'c':
            send_zoom_stop();
            break;

        // ★ ПЕРЕКЛЮЧЕНИЕ МЕЖДУ ДВУМЯ ПОТОКАМИ ★
        case '1':
            set_image_mode(vdisp_rgb_, "RGB ЗУМ");
            {
                auto msg = std_msgs::msg::Int32();
                msg.data = 1; // video2
                stream_publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "📺 Основной поток video2 (RGB ЗУМ)");
            }
            break;
        case '2':
            set_image_mode(vdisp_wide_thermal_, "WIDE или THERMAL");
            {
                auto msg = std_msgs::msg::Int32();
                msg.data = 0; // video1
                stream_publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "📺 Дополнительный поток video1 (WIDE или THERMAL)");
            }
            break;

        // Настройки
        case '+': case '=':
            if (step_ < 20.0) {
                step_ += 0.5;
                RCLCPP_INFO(this->get_logger(), "📏 Шаг: %.1f°", step_);
            }
            break;
        case '-': case '_':
            if (step_ > 0.5) {
                step_ -= 0.5;
                RCLCPP_INFO(this->get_logger(), "📏 Шаг: %.1f°", step_);
            }
            break;
        case 'H': case 'h':
            print_help();
            break;
        default:
            break;
        }

        if (command_sent) {
            pitch_ = new_pitch;
            yaw_ = new_yaw;
            send_gimbal_command(pitch_, yaw_);
        }
    }

    // ============================================
    // ПЕРЕМЕННЫЕ
    // ============================================
    std::string publish_topic_;
    std::string angle_publish_topic_;
    std::string service_name_;
    int gimbal_device_id_;
    int flags_;
    double step_;
    double max_pitch_;
    double min_pitch_;
    double max_yaw_;
    double min_yaw_;
    int keyboard_poll_rate_hz_;
    double service_timeout_sec_;
    double zoom_speed_;

    std::string camera_ip_;
    int camera_port_;
    int vdisp_rgb_, vdisp_wide_thermal_;

    rclcpp::Publisher<mavros_msgs::msg::GimbalManagerSetPitchyaw>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr angle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr stream_publisher_;
    rclcpp::Client<mavros_msgs::srv::GimbalManagerPitchyaw>::SharedPtr client_;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr cmd_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    struct termios old_termios_;
    double pitch_ = 0.0f;
    double yaw_ = 0.0f;
    std::atomic<bool> running_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<KeyboardGimbalController>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "❌ Ошибка: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}