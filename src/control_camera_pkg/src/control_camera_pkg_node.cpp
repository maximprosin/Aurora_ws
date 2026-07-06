#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <sstream>

class KeyboardTeleop : public rclcpp::Node {
public:
    KeyboardTeleop() : Node("keyboard_teleop"), running_(true) {
        // ============================================
        // 1. ПАРАМЕТРЫ
        // ============================================
        this->declare_parameter("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter("data_topic", "/teleop/data");
        this->declare_parameter("linear_speed", 0.5);
        this->declare_parameter("angular_speed", 1.0);

        cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
        data_topic_ = this->get_parameter("data_topic").as_string();
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        angular_speed_ = this->get_parameter("angular_speed").as_double();

        // ============================================
        // 2. ПУБЛИКАТОРЫ
        // ============================================
        // Основной публикатор для движения (MAVROS подписан)
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

        // Дополнительный публикатор для данных (например, для TCP-моста)
        data_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(data_topic_, 10);

        // ============================================
        // 3. ВЫВОД СПРАВКИ
        // ============================================
        RCLCPP_INFO(this->get_logger(), "======================================");
        RCLCPP_INFO(this->get_logger(), "🎮 Управление гусеничной тележкой");
        RCLCPP_INFO(this->get_logger(), "======================================");
        RCLCPP_INFO(this->get_logger(), "W - Вперёд");
        RCLCPP_INFO(this->get_logger(), "S - Назад");
        RCLCPP_INFO(this->get_logger(), "A - Поворот налево");
        RCLCPP_INFO(this->get_logger(), "D - Поворот направо");
        RCLCPP_INFO(this->get_logger(), "Q - Танковый разворот налево");
        RCLCPP_INFO(this->get_logger(), "E - Танковый разворот направо");
        RCLCPP_INFO(this->get_logger(), "Space - Остановка");
        RCLCPP_INFO(this->get_logger(), "======================================");
        RCLCPP_INFO(this->get_logger(), "📤 Публикация в:");
        RCLCPP_INFO(this->get_logger(), "   - %s (для MAVROS)", cmd_vel_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "   - %s (для TCP/моста)", data_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "======================================");
        RCLCPP_INFO(this->get_logger(), "Нажмите Ctrl+C для выхода");
        RCLCPP_INFO(this->get_logger(), "======================================");

        // Запускаем поток для чтения клавиш
        input_thread_ = std::thread(&KeyboardTeleop::read_keys, this);
    }

    ~KeyboardTeleop() {
        running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    // ============================================
    // ПУБЛИКАЦИЯ ДАННЫХ
    // ============================================
    void publish_data(double linear_x, double angular_z) {
        // 1. Публикация в Twist (для MAVROS)
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = linear_x;
        twist_msg.angular.z = angular_z;
        cmd_pub_->publish(twist_msg);

        // 2. Публикация в Float32MultiArray (для TCP/моста)
        auto data_msg = std_msgs::msg::Float32MultiArray();
        data_msg.data.clear();
        data_msg.data.push_back(static_cast<float>(linear_x));
        data_msg.data.push_back(static_cast<float>(angular_z));
        data_pub_->publish(data_msg);

        // Логирование
        RCLCPP_DEBUG(this->get_logger(), "📤 Data: linear=%.2f, angular=%.2f", linear_x, angular_z);
    }

    // ============================================
    // ЧТЕНИЕ КЛАВИАТУРЫ
    // ============================================
    void read_keys() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

        char key;
        double linear = 0.0;
        double angular = 0.0;

        while (running_ && rclcpp::ok()) {
            if (read(STDIN_FILENO, &key, 1) > 0) {
                linear = 0.0;
                angular = 0.0;

                switch (key) {
                case 'w':
                case 'W':
                    linear = linear_speed_;
                    RCLCPP_INFO(this->get_logger(), "⬆️ Вперёд");
                    break;

                case 's':
                case 'S':
                    linear = -linear_speed_;
                    RCLCPP_INFO(this->get_logger(), "⬇️ Назад");
                    break;

                case 'a':
                case 'A':
                    linear = linear_speed_ * 0.5;
                    angular = angular_speed_;
                    RCLCPP_INFO(this->get_logger(), "↰ Поворот налево");
                    break;

                case 'd':
                case 'D':
                    linear = linear_speed_ * 0.5;
                    angular = -angular_speed_;
                    RCLCPP_INFO(this->get_logger(), "↱ Поворот направо");
                    break;

                case 'q':
                case 'Q':
                    angular = angular_speed_;
                    RCLCPP_INFO(this->get_logger(), "🔄 Танковый разворот налево");
                    break;

                case 'e':
                case 'E':
                    angular = -angular_speed_;
                    RCLCPP_INFO(this->get_logger(), "🔄 Танковый разворот направо");
                    break;

                case ' ':
                    linear = 0.0;
                    angular = 0.0;
                    RCLCPP_INFO(this->get_logger(), "🛑 СТОП!");
                    break;

                default:
                    continue;
                }

                // Публикуем данные
                publish_data(linear, angular);
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
    double linear_speed_;
    double angular_speed_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr data_pub_;
    std::thread input_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
