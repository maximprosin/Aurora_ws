#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <algorithm>

class KeyboardTeleop : public rclcpp::Node {
public:
    KeyboardTeleop() : Node("keyboard_teleop"), running_(true), pixhawk_connected_(false) {
        // ============================================
        // 1. ПАРАМЕТРЫ
        // ============================================
        this->declare_parameter("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter("data_topic", "/teleop/data");
        this->declare_parameter("key_topic", "/teleop/key");
        this->declare_parameter("linear_speed", 0.5);
        this->declare_parameter("angular_speed", 1.0);
        this->declare_parameter("speed_step", 0.05);
        this->declare_parameter("log_level", "info");

        cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
        data_topic_ = this->get_parameter("data_topic").as_string();
        key_topic_ = this->get_parameter("key_topic").as_string();
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        angular_speed_ = this->get_parameter("angular_speed").as_double();
        speed_step_ = this->get_parameter("speed_step").as_double();
        log_level_ = this->get_parameter("log_level").as_string();

        // ============================================
        // 2. ПУБЛИКАТОРЫ
        // ============================================
        // Для движения (MAVROS)
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

        // Для TCP/моста (скорости)
        data_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(data_topic_, 10);

        // Для отладки/логирования (нажатые клавиши)
        key_pub_ = this->create_publisher<std_msgs::msg::String>(key_topic_, 10);

        // ============================================
        // 3. ПОДПИСКА НА СТАТУС MAVROS
        // ============================================
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&KeyboardTeleop::state_callback, this, std::placeholders::_1));

        // ============================================
        // 4. ТАЙМЕР ДЛЯ ПРОВЕРКИ ПОДКЛЮЧЕНИЯ
        // ============================================
        check_connection_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&KeyboardTeleop::check_connection, this));

        // ============================================
        // 5. СТАТИСТИКА
        // ============================================
        last_publish_time_ = this->now();
        command_count_ = 0;

        // ============================================
        // 6. ВЫВОД СПРАВКИ
        // ============================================
        print_help();

        // ============================================
        // 7. ТАЙМЕР ДЛЯ СТАТИСТИКИ
        // ============================================
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&KeyboardTeleop::print_stats, this));

        // ============================================
        // 8. ЗАПУСК ПОТОКА КЛАВИАТУРЫ
        // ============================================
        input_thread_ = std::thread(&KeyboardTeleop::read_keys, this);

        RCLCPP_INFO(this->get_logger(), "✅ Узел управления запущен");
        RCLCPP_INFO(this->get_logger(), "📤 Публикация в:");
        RCLCPP_INFO(this->get_logger(), "   - %s (для MAVROS)", cmd_vel_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "   - %s (для TCP/моста)", data_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "   - %s (для клавиш)", key_topic_.c_str());
    }

    ~KeyboardTeleop() {
        running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "🛑 Узел остановлен");
    }

private:
    // ============================================
    // КОЛБЭК СТАТУСА MAVROS
    // ============================================
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
        current_state_ = *msg;
        pixhawk_connected_ = msg->connected;

        if (msg->connected) {
            RCLCPP_DEBUG(this->get_logger(), "✅ Pixhawk подключен");
        } else {
            RCLCPP_WARN(this->get_logger(), "❌ Pixhawk НЕ подключен");
        }
    }

    // ============================================
    // ПРОВЕРКА ПОДКЛЮЧЕНИЯ
    // ============================================
    void check_connection() {
        if (!pixhawk_connected_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "⚠️ Ожидание подключения к Pixhawk...");
        }
    }

    // ============================================
    // СТАТУС PIXHAWK В ВИДЕ КРАСИВОЙ СТРОКИ
    // ============================================
    std::string get_pixhawk_status() {
        std::stringstream ss;

        if (pixhawk_connected_) {
            ss << "✅ ПОДКЛЮЧЕН";

            if (current_state_.armed) {
                ss << " [ВЗВЕДЕН]";
            } else {
                ss << " [НЕ ВЗВЕДЕН]";
            }

            if (!current_state_.mode.empty()) {
                ss << " [" << current_state_.mode << "]";
            }

            if (current_state_.system_status == 0) {
                ss << " [НОРМА]";
            } else if (current_state_.system_status == 1) {
                ss << " [КАЛИБРОВКА]";
            } else if (current_state_.system_status == 4) {
                ss << " [ОШИБКА]";
            }

        } else {
            ss << "❌ НЕ ПОДКЛЮЧЕН";
            ss << " (проверьте USB/телерию)";
        }

        return ss.str();
    }

    // ============================================
    // ВЫВОД СПРАВКИ
    // ============================================
    void print_help() {
        std::stringstream ss;
        std::string status = get_pixhawk_status();

        ss << "\n";
        ss << "╔══════════════════════════════════════════════════════════════════════════╗\n";
        ss << "║                    🚜 УПРАВЛЕНИЕ ГУСЕНИЧНОЙ ТЕЛЕЖКОЙ                   ║\n";
        ss << "╠══════════════════════════════════════════════════════════════════════════╣\n";

        ss << "║                                                                          ║\n";
        ss << "║  ╔══════════════════════════════════════════════════════════════════╗     ║\n";
        ss << "║  ║   🔌 СТАТУС PIXHAWK: " << std::left << std::setw(35) << status << "║     ║\n";
        ss << "║  ╚══════════════════════════════════════════════════════════════════╝     ║\n";

        ss << "║                                                                          ║\n";
        ss << "║  ╔══════════════════════════════════════════════════════════════════╗     ║\n";
        ss << "║  ║                     🎮 УПРАВЛЕНИЕ ДВИЖЕНИЕМ                    ║     ║\n";
        ss << "║  ╠══════════════════════════════════════════════════════════════════╣     ║\n";
        ss << "║  ║                                                                  ║     ║\n";
        ss << "║  ║                      [  ↑  ]                                    ║     ║\n";
        ss << "║  ║                   W - Вперёд                                    ║     ║\n";
        ss << "║  ║                                                                  ║     ║\n";
        ss << "║  ║            [  ←  ]        [  →  ]                              ║     ║\n";
        ss << "║  ║         A - Налево    D - Направо                              ║     ║\n";
        ss << "║  ║                                                                  ║     ║\n";
        ss << "║  ║   [  ↺  ]              [  ↻  ]                                 ║     ║\n";
        ss << "║  ║ Q - Разворот налево  E - Разворот направо                     ║     ║\n";
        ss << "║  ║   (на месте)           (на месте)                               ║     ║\n";
        ss << "║  ║                                                                  ║     ║\n";
        ss << "║  ║                      [  ↓  ]                                    ║     ║\n";
        ss << "║  ║                   S - Назад                                     ║     ║\n";
        ss << "║  ║                                                                  ║     ║\n";
        ss << "║  ║   ╔═══════════════════════════════════════════════════════════╗  ║     ║\n";
        ss << "║  ║   ║   [ SPACE ] - ЭКСТРЕННАЯ ОСТАНОВКА (Стоп-кран)          ║  ║     ║\n";
        ss << "║  ║   ╚═══════════════════════════════════════════════════════════╝  ║     ║\n";
        ss << "║  ╚══════════════════════════════════════════════════════════════════╝     ║\n";
        ss << "║                                                                          ║\n";
        ss << "║  ╔══════════════════════════════════════════════════════════════════╗     ║\n";
        ss << "║  ║                    ⚡ НАСТРОЙКА СКОРОСТИ                      ║     ║\n";
        ss << "║  ╠══════════════════════════════════════════════════════════════════╣     ║\n";
        ss << "║  ║                                                                  ║     ║\n";
        ss << "║  ║   [ R ] - Увеличить скорость вперёд    [ F ] - Уменьшить       ║     ║\n";
        ss << "║  ║   [ T ] - Увеличить скорость поворота   [ G ] - Уменьшить       ║     ║\n";
        ss << "║  ║                                                                  ║     ║\n";
        ss << "║  ║   ТЕКУЩАЯ СКОРОСТЬ:                                             ║     ║\n";
        ss << "║  ║   ┌─────────────────────────────────────────────────────────┐    ║     ║\n";
        ss << "║  ║   │  Линейная: " << std::setw(5) << std::fixed << std::setprecision(2) << linear_speed_ << " м/с    │    ║     ║\n";
        ss << "║  ║   │  Угловая:  " << std::setw(5) << std::fixed << std::setprecision(2) << angular_speed_ << " рад/с   │    ║     ║\n";
        ss << "║  ║   └─────────────────────────────────────────────────────────┘    ║     ║\n";
        ss << "║  ╚══════════════════════════════════════════════════════════════════╝     ║\n";
        ss << "║                                                                          ║\n";
        ss << "║  ╔══════════════════════════════════════════════════════════════════╗     ║\n";
        ss << "║  ║                    ℹ️ ИНФОРМАЦИЯ                              ║     ║\n";
        ss << "║  ╠══════════════════════════════════════════════════════════════════╣     ║\n";
        ss << "║  ║                                                                  ║     ║\n";
        ss << "║  ║   [ H ] - Показать эту справку    [ P ] - Показать параметры    ║     ║\n";
        ss << "║  ║                                                                  ║     ║\n";
        ss << "║  ║   [ Ctrl+C ] - Выход                                             ║     ║\n";
        ss << "║  ║                                                                  ║     ║\n";
        ss << "║  ║   📤 Клавиши публикуются в: " << key_topic_ << "                  ║     ║\n";
        ss << "║  ╚══════════════════════════════════════════════════════════════════╝     ║\n";
        ss << "║                                                                          ║\n";

        if (!pixhawk_connected_) {
            ss << "║  ╔══════════════════════════════════════════════════════════════════╗     ║\n";
            ss << "║  ║   ⚠️  ВНИМАНИЕ: Pixhawk НЕ ПОДКЛЮЧЕН!                         ║     ║\n";
            ss << "║  ║   Управление тележкой НЕВОЗМОЖНО!                              ║     ║\n";
            ss << "║  ║   Проверьте:                                                   ║     ║\n";
            ss << "║  ║   - USB-кабель                                                 ║     ║\n";
            ss << "║  ║   - Питание Pixhawk                                            ║     ║\n";
            ss << "║  ║   - MAVROS (ros2 run mavros apm.launch.py)                    ║     ║\n";
            ss << "║  ╚══════════════════════════════════════════════════════════════════╝     ║\n";
        }

        ss << "╚══════════════════════════════════════════════════════════════════════════╝\n";

        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    // ============================================
    // ПОКАЗАТЬ ТЕКУЩИЕ ПАРАМЕТРЫ
    // ============================================
    void print_params() {
        std::stringstream ss;
        std::string status = get_pixhawk_status();

        ss << "\n";
        ss << "╔══════════════════════════════════════════════════════════════╗\n";
        ss << "║              📊 ТЕКУЩИЕ ПАРАМЕТРЫ                          ║\n";
        ss << "╠══════════════════════════════════════════════════════════════╣\n";
        ss << "║                                                              ║\n";
        ss << "║  🔌 Pixhawk: " << status << "\n";
        ss << "║                                                              ║\n";

        int bar_width = 20;
        int linear_bars = static_cast<int>((linear_speed_ / 2.0) * bar_width);
        int angular_bars = static_cast<int>((angular_speed_ / 3.0) * bar_width);

        ss << "║  Линейная скорость:  " << std::fixed << std::setprecision(2) << linear_speed_ << " м/с\n";
        ss << "║  [";
        for (int i = 0; i < bar_width; i++) {
            ss << (i < linear_bars ? "█" : "░");
        }
        ss << "]\n";
        ss << "║                                                              ║\n";
        ss << "║  Угловая скорость:   " << std::fixed << std::setprecision(2) << angular_speed_ << " рад/с\n";
        ss << "║  [";
        for (int i = 0; i < bar_width; i++) {
            ss << (i < angular_bars ? "█" : "░");
        }
        ss << "]\n";
        ss << "║                                                              ║\n";
        ss << "║  Шаг изменения:      " << std::fixed << std::setprecision(2) << speed_step_ << "\n";
        ss << "║  Кол-во команд:      " << command_count_ << "\n";
        ss << "║  Частота публикации: " << std::fixed << std::setprecision(1) << get_publish_rate() << " Гц\n";
        ss << "║  Топик клавиш:       " << key_topic_ << "\n";

        if (!pixhawk_connected_) {
            ss << "║                                                              ║\n";
            ss << "║  ⚠️ Pixhawk НЕ ПОДКЛЮЧЕН! Команды не будут выполняться!      ║\n";
        }

        ss << "║                                                              ║\n";
        ss << "╚══════════════════════════════════════════════════════════════╝\n";

        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    // ============================================
    // СТАТИСТИКА
    // ============================================
    double get_publish_rate() {
        auto now = this->now();
        double dt = (now - last_publish_time_).seconds();
        if (dt > 0) {
            return command_count_ / dt;
        }
        return 0.0;
    }

    void print_stats() {
        if (command_count_ > 0) {
            double rate = get_publish_rate();
            std::string status = pixhawk_connected_ ? "✅" : "❌";
            RCLCPP_INFO(this->get_logger(), "📊 %s %d команд, %.1f Гц",
                        status.c_str(), command_count_, rate);
            command_count_ = 0;
            last_publish_time_ = this->now();
        }
    }

    // ============================================
    // ПУБЛИКАЦИЯ ДАННЫХ
    // ============================================
    void publish_key(const std::string& key_name, double linear_x, double angular_z) {
        // 1. Публикация клавиши в топик String
        auto key_msg = std_msgs::msg::String();
        key_msg.data = key_name;
        key_pub_->publish(key_msg);

        // 2. Публикация скорости (как было)
        if (!pixhawk_connected_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "⚠️ Pixhawk не подключен! Команда отправлена, но может не выполниться");
        }

        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = linear_x;
        twist_msg.angular.z = angular_z;
        cmd_pub_->publish(twist_msg);

        auto data_msg = std_msgs::msg::Float32MultiArray();
        data_msg.data.clear();
        data_msg.data.push_back(static_cast<float>(linear_x));
        data_msg.data.push_back(static_cast<float>(angular_z));
        data_pub_->publish(data_msg);

        command_count_++;

        if (log_level_ == "debug") {
            RCLCPP_DEBUG(this->get_logger(), "📤 Key: %s, linear=%.2f, angular=%.2f",
                         key_name.c_str(), linear_x, angular_z);
        }
    }

    // ============================================
    // БЕЗОПАСНАЯ УСТАНОВКА СКОРОСТИ
    // ============================================
    void set_linear_speed(double new_speed) {
        if (new_speed < 0.05) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Минимальная линейная скорость: 0.05 м/с");
            linear_speed_ = 0.05;
        } else if (new_speed > 2.0) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Максимальная линейная скорость: 2.0 м/с");
            linear_speed_ = 2.0;
        } else {
            linear_speed_ = new_speed;
            RCLCPP_INFO(this->get_logger(), "✅ Линейная скорость: %.2f м/с", linear_speed_);
        }
    }

    void set_angular_speed(double new_speed) {
        if (new_speed < 0.1) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Минимальная угловая скорость: 0.1 рад/с");
            angular_speed_ = 0.1;
        } else if (new_speed > 3.0) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Максимальная угловая скорость: 3.0 рад/с");
            angular_speed_ = 3.0;
        } else {
            angular_speed_ = new_speed;
            RCLCPP_INFO(this->get_logger(), "✅ Угловая скорость: %.2f рад/с", angular_speed_);
        }
    }

    // ============================================
    // ЧТЕНИЕ КЛАВИАТУРЫ
    // ============================================
    void read_keys() {
        struct termios oldt, newt;
        if (tcgetattr(STDIN_FILENO, &oldt) != 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Не удалось получить настройки терминала");
            return;
        }
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_cc[VMIN] = 0;
        newt.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) != 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Не удалось установить настройки терминала");
            return;
        }

        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags == -1 || fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) == -1) {
            RCLCPP_ERROR(this->get_logger(), "❌ Не удалось сделать STDIN неблокирующим");
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
            return;
        }

        char key;
        double linear = 0.0;
        double angular = 0.0;
        std::string key_name;
        bool key_pressed = false;

        while (running_ && rclcpp::ok()) {
            if (read(STDIN_FILENO, &key, 1) > 0) {
                key_pressed = true;
                linear = 0.0;
                angular = 0.0;

                switch (key) {
                case 'w': case 'W':
                    linear = linear_speed_;
                    key_name = "W";
                    RCLCPP_INFO(this->get_logger(), "⬆️ Вперёд [%.2f м/с]", linear);
                    break;

                case 's': case 'S':
                    linear = -linear_speed_;
                    key_name = "S";
                    RCLCPP_INFO(this->get_logger(), "⬇️ Назад [%.2f м/с]", linear);
                    break;

                case 'a': case 'A':
                    linear = linear_speed_ * 0.6;
                    angular = angular_speed_;
                    key_name = "A";
                    RCLCPP_INFO(this->get_logger(), "↰ Поворот налево [lin=%.2f, ang=%.2f]", linear, angular);
                    break;

                case 'd': case 'D':
                    linear = linear_speed_ * 0.6;
                    angular = -angular_speed_;
                    key_name = "D";
                    RCLCPP_INFO(this->get_logger(), "↱ Поворот направо [lin=%.2f, ang=%.2f]", linear, angular);
                    break;

                case 'q': case 'Q':
                    angular = angular_speed_;
                    key_name = "Q";
                    RCLCPP_INFO(this->get_logger(), "🔄 Разворот налево [ang=%.2f]", angular);
                    break;

                case 'e': case 'E':
                    angular = -angular_speed_;
                    key_name = "E";
                    RCLCPP_INFO(this->get_logger(), "🔄 Разворот направо [ang=%.2f]", angular);
                    break;

                case ' ':
                    linear = 0.0;
                    angular = 0.0;
                    key_name = "SPACE";
                    RCLCPP_INFO(this->get_logger(), "🛑 СТОП!");
                    break;

                case 'r': case 'R':
                    set_linear_speed(linear_speed_ + speed_step_);
                    continue;

                case 'f': case 'F':
                    set_linear_speed(linear_speed_ - speed_step_);
                    continue;

                case 't': case 'T':
                    set_angular_speed(angular_speed_ + speed_step_);
                    continue;

                case 'g': case 'G':
                    set_angular_speed(angular_speed_ - speed_step_);
                    continue;

                case 'h': case 'H':
                    print_help();
                    continue;

                case 'p': case 'P':
                    print_params();
                    continue;

                default:
                    key_pressed = false;
                    continue;
                }

                publish_key(key_name, linear, angular);

            } else {
                if (key_pressed) {
                    key_pressed = false;
                    key_name = "RELEASE";
                    publish_key(key_name, 0.0, 0.0);
                    RCLCPP_DEBUG(this->get_logger(), "⏹️ Остановка (клавиша отпущена)");
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

    // ============================================
    // ПЕРЕМЕННЫЕ
    // ============================================
    std::string cmd_vel_topic_;
    std::string data_topic_;
    std::string key_topic_;
    std::string log_level_;
    double linear_speed_;
    double angular_speed_;
    double speed_step_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr data_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr key_pub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::TimerBase::SharedPtr check_connection_timer_;
    std::thread input_thread_;
    std::atomic<bool> running_;

    mavros_msgs::msg::State current_state_;
    bool pixhawk_connected_;

    rclcpp::Time last_publish_time_;
    int command_count_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<KeyboardTeleop>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "❌ Критическая ошибка: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
