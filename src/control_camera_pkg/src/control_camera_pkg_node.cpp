#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/gimbal_manager_pitchyaw.hpp>
#include <mavros_msgs/msg/gimbal_manager_set_pitchyaw.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>  // Для публикации углов
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <algorithm>
#include <atomic>
#include <csignal>
#include <sstream>
#include <iomanip>

class KeyboardGimbalController : public rclcpp::Node {
public:
    KeyboardGimbalController() : Node("keyboard_gimbal_controller"), running_(true) {
        // ============================================
        // 1. ПАРАМЕТРИЗАЦИЯ
        // ============================================
        this->declare_parameter("publish_topic", "/gimbal/commands");
        this->declare_parameter("angle_publish_topic", "/gimbal/angles");  // Новый топик для углов
        this->declare_parameter("service_name", "/mavros/gimbal_control/manager/pitchyaw");
        this->declare_parameter("gimbal_device_id", 1);
        this->declare_parameter("flags", 0);
        this->declare_parameter("step_angle", 2.0f);
        this->declare_parameter("max_pitch", 25.0f);   // Аппаратный максимум +25°
        this->declare_parameter("min_pitch", -90.0f);  // Аппаратный минимум -90°
        this->declare_parameter("max_yaw", 180.0f);
        this->declare_parameter("min_yaw", -180.0f);
        this->declare_parameter("keyboard_poll_rate_hz", 50);
        this->declare_parameter("service_timeout_sec", 5.0);

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

        // ============================================
        // 2. ПУБЛИКАТОРЫ
        // ============================================
        // Публикатор команд для подвеса
        publisher_ = this->create_publisher<mavros_msgs::msg::GimbalManagerSetPitchyaw>(
            publish_topic_, 10);

        // Публикатор углов для видео потока (Float32MultiArray)
        angle_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            angle_publish_topic_, 10);

        // ============================================
        // 3. КЛИЕНТ ДЛЯ СЕРВИСА MAVROS
        // ============================================
        client_ = this->create_client<mavros_msgs::srv::GimbalManagerPitchyaw>(service_name_);

        if (!wait_for_service_with_timeout()) {
            RCLCPP_ERROR(this->get_logger(),
                         "❌ Сервис %s не доступен после %.1f сек.",
                         service_name_.c_str(), service_timeout_sec_);
        } else {
            RCLCPP_INFO(this->get_logger(), "✅ Сервис %s доступен", service_name_.c_str());
        }

        // ============================================
        // 4. НАСТРОЙКА ТЕРМИНАЛА
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
        // 6. ВЫВОД ПРИВЕТСТВИЯ
        // ============================================
        RCLCPP_INFO(this->get_logger(), "=== Узел управления камерой запущен ===");
        RCLCPP_INFO(this->get_logger(), "📊 Текущие параметры:");
        RCLCPP_INFO(this->get_logger(), "  Шаг: %.1f°", step_);
        RCLCPP_INFO(this->get_logger(), "  Наклон (pitch): [%.1f° .. %.1f°]", min_pitch_, max_pitch_);
        RCLCPP_INFO(this->get_logger(), "  Поворот (yaw): [%.1f° .. %.1f°]", min_yaw_, max_yaw_);
        RCLCPP_INFO(this->get_logger(), "  Публикация углов в топик: %s", angle_publish_topic_.c_str());
        print_help();
    }

    ~KeyboardGimbalController() {
        running_ = false;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_) == 0) {
            RCLCPP_DEBUG(this->get_logger(), "Настройки терминала восстановлены");
        }
    }

private:
    void print_help() {
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "📋 Управление камерой:");
        RCLCPP_INFO(this->get_logger(), "  W/S - наклон вверх/вниз (pitch)");
        RCLCPP_INFO(this->get_logger(), "  A/D - поворот влево/вправо (yaw)");
        RCLCPP_INFO(this->get_logger(), "  ПРОБЕЛ - сброс камеры в центр (0°, 0°)");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "📋 Настройки параметров:");
        RCLCPP_INFO(this->get_logger(), "  + / - - увеличить/уменьшить шаг (от 0.5° до 20°)");
        RCLCPP_INFO(this->get_logger(), "  ESC - выход");
        RCLCPP_INFO(this->get_logger(), "  H/h - показать эту справку");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "📊 Текущий шаг: %.1f°", step_);
        RCLCPP_INFO(this->get_logger(), "📤 Углы публикуются в: %s", angle_publish_topic_.c_str());
    }

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

    // ============================================
    // ПУБЛИКАЦИЯ УГЛОВ В ОТДЕЛЬНЫЙ ТОПИК
    // ============================================
    void publish_angles(float pitch, float yaw) {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data.clear();
        msg.data.push_back(static_cast<float>(pitch));
        msg.data.push_back(static_cast<float>(yaw));
        // Можно добавить и другие данные, например:
        // msg.data.push_back(static_cast<float>(step_)); // Шаг
        // msg.data.push_back(static_cast<float>(max_pitch_)); // Максимальный наклон
        angle_publisher_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "📤 Опубликованы углы: Pitch=%.1f, Yaw=%.1f", pitch, yaw);
    }

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

        // 1. Отправляем команду на Pixhawk
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
                                                            "❌ Команда не выполнена: result=%d", result->result);
                                            } else {
                                                RCLCPP_DEBUG(this->get_logger(), "✅ Команда выполнена");
                                            }
                                        });
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ Сервис MAVROS не доступен.");
        }

        // 2. Публикуем команду в топик /gimbal/commands
        auto msg = mavros_msgs::msg::GimbalManagerSetPitchyaw();
        msg.pitch = pitch;
        msg.yaw = yaw;
        msg.pitch_rate = 0.0f;
        msg.yaw_rate = 0.0f;
        msg.flags = flags_;
        msg.gimbal_device_id = gimbal_device_id_;
        publisher_->publish(msg);

        // 3. Публикуем углы в отдельный топик для видео потока
        publish_angles(pitch, yaw);
    }

    void read_keyboard() {
        int key = get_key();
        if (key == -1) return;

        float new_pitch = pitch_;
        float new_yaw = yaw_;
        bool command_sent = false;

        switch (key) {
        case 'W':
        case 'w':
            new_pitch = pitch_ + step_;
            if (new_pitch > max_pitch_) {
                new_pitch = max_pitch_;
                RCLCPP_INFO(this->get_logger(), "🔒 Достигнут максимум наклона: %.1f°", new_pitch);
            } else {
                RCLCPP_INFO(this->get_logger(), "📐 Наклон вверх: %.1f°", new_pitch);
            }
            command_sent = true;
            break;
        case 'S':
        case 's':
            new_pitch = pitch_ - step_;
            if (new_pitch < min_pitch_) {
                new_pitch = min_pitch_;
                RCLCPP_INFO(this->get_logger(), "🔒 Достигнут минимум наклона: %.1f°", new_pitch);
            } else {
                RCLCPP_INFO(this->get_logger(), "📐 Наклон вниз: %.1f°", new_pitch);
            }
            command_sent = true;
            break;
        case 'A':
        case 'a':
            new_yaw = yaw_ + step_;
            if (new_yaw > max_yaw_) {
                new_yaw = max_yaw_;
                RCLCPP_INFO(this->get_logger(), "🔒 Достигнут максимум поворота: %.1f°", new_yaw);
            } else {
                RCLCPP_INFO(this->get_logger(), "📐 Поворот влево: %.1f°", new_yaw);
            }
            command_sent = true;
            break;
        case 'D':
        case 'd':
            new_yaw = yaw_ - step_;
            if (new_yaw < min_yaw_) {
                new_yaw = min_yaw_;
                RCLCPP_INFO(this->get_logger(), "🔒 Достигнут минимум поворота: %.1f°", new_yaw);
            } else {
                RCLCPP_INFO(this->get_logger(), "📐 Поворот вправо: %.1f°", new_yaw);
            }
            command_sent = true;
            break;
        case ' ':  // ПРОБЕЛ
            new_pitch = 0.0f;
            new_yaw = 0.0f;
            RCLCPP_INFO(this->get_logger(), "🔄 Сброс камеры в центр (0°, 0°)");
            command_sent = true;
            break;
        case '+':
        case '=':
            if (step_ < 20.0) {
                step_ += 0.5;
                RCLCPP_INFO(this->get_logger(), "📏 Скорость увеличена. Новый шаг: %.1f°", step_);
            } else {
                RCLCPP_INFO(this->get_logger(), "⚠️ Шаг уже максимальный (20.0°)");
            }
            break;
        case '-':
        case '_':
            if (step_ > 0.5) {
                step_ -= 0.5;
                RCLCPP_INFO(this->get_logger(), "📏 Скорость уменьшена. Новый шаг: %.1f°", step_);
            } else {
                RCLCPP_INFO(this->get_logger(), "⚠️ Шаг уже минимальный (0.5°)");
            }
            break;
        case 'H':
        case 'h':
            print_help();
            break;
        case 27:  // ESC
            RCLCPP_INFO(this->get_logger(), "⏹️ Выход...");
            running_ = false;
            rclcpp::shutdown();
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

    rclcpp::Publisher<mavros_msgs::msg::GimbalManagerSetPitchyaw>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr angle_publisher_;
    rclcpp::Client<mavros_msgs::srv::GimbalManagerPitchyaw>::SharedPtr client_;
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
        RCLCPP_ERROR(rclcpp::get_logger("main"), "❌ Критическая ошибка: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
