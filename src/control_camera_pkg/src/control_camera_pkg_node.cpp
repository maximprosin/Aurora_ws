#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/gimbal_manager_pitchyaw.hpp>
#include <mavros_msgs/msg/gimbal_manager_set_pitchyaw.hpp>  // Добавлено для публикации
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <map>

class KeyboardGimbalController : public rclcpp::Node {
public:
    KeyboardGimbalController() : Node("keyboard_gimbal_controller") {
        // ПУБЛИКАТОР: создаём издателя в топик /gimbal/commands
        publisher_ = this->create_publisher<mavros_msgs::msg::GimbalManagerSetPitchyaw>(
            "/gimbal/commands", 10);

        // Создаём клиент для сервиса управления подвесом (на Pixhawk)
        client_ = this->create_client<mavros_msgs::srv::GimbalManagerPitchyaw>(
            "/mavros/gimbal_control/manager/pitchyaw");

        // Ждём, пока сервис MAVROS станет доступен
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Ожидание сервиса /mavros/gimbal_control/manager/pitchyaw...");
        }
        RCLCPP_INFO(this->get_logger(), "Узел управления камерой запущен.");
        RCLCPP_INFO(this->get_logger(), "Управление:");
        RCLCPP_INFO(this->get_logger(), "  W/S - наклон камеры (pitch)");
        RCLCPP_INFO(this->get_logger(), "  A/D - поворот камеры (yaw)");
        RCLCPP_INFO(this->get_logger(), "  ПРОБЕЛ - сброс в центр");
        RCLCPP_INFO(this->get_logger(), "  ESC - выход");
        RCLCPP_INFO(this->get_logger(), "Команды публикуются в топик /gimbal/commands");

        // Настраиваем терминал для неблокирующего ввода
        setup_terminal();

        // Создаём таймер для опроса клавиатуры (50 Гц)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&KeyboardGimbalController::read_keyboard, this));
    }

    ~KeyboardGimbalController() {
        // Восстанавливаем настройки терминала при выходе
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
    }

private:
    void setup_terminal() {
        // Сохраняем старые настройки терминала
        tcgetattr(STDIN_FILENO, &old_termios_);
        struct termios new_termios = old_termios_;

        // Отключаем канонический режим и эхо
        new_termios.c_lflag &= ~(ICANON | ECHO);
        // Устанавливаем минимальное чтение
        new_termios.c_cc[VMIN] = 0;
        new_termios.c_cc[VTIME] = 0;

        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

        // Делаем STDIN неблокирующим
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    int get_key() {
        // Неблокирующее чтение одного символа
        char ch;
        if (read(STDIN_FILENO, &ch, 1) == 1) {
            return ch;
        }
        return -1;
    }

    // ОБНОВЛЁННАЯ ФУНКЦИЯ: теперь и отправляет на Pixhawk, и публикует в топик
    void send_gimbal_command(float pitch, float yaw) {
        // 1. Отправляем команду на Pixhawk через сервис
        auto request = std::make_shared<mavros_msgs::srv::GimbalManagerPitchyaw::Request>();
        request->pitch = pitch;
        request->yaw = yaw;
        request->pitch_rate = 0.0f;
        request->yaw_rate = 0.0f;
        request->flags = 0;  // 0 = FOLLOW (относительно корпуса), 16 = LOCK
        request->gimbal_device_id = 1;  // ID вашего подвеса

        client_->async_send_request(request);

        // 2. Публикуем команду в топик /gimbal/commands (для TCP-сервера и других подписчиков)
        auto msg = mavros_msgs::msg::GimbalManagerSetPitchyaw();
        msg.pitch = pitch;
        msg.yaw = yaw;
        msg.pitch_rate = 0.0f;
        msg.yaw_rate = 0.0f;
        msg.flags = 0;
        msg.gimbal_device_id = 1;

        publisher_->publish(msg);
    }

    void read_keyboard() {
        int key = get_key();
        if (key == -1) return;

        // Обработка клавиш
        switch (key) {
        case 'W':
        case 'w':
            pitch_ += step_;
            RCLCPP_INFO(this->get_logger(), "Наклон: %.1f", pitch_);
            send_gimbal_command(pitch_, yaw_);
            break;
        case 'S':
        case 's':
            pitch_ -= step_;
            RCLCPP_INFO(this->get_logger(), "Наклон: %.1f", pitch_);
            send_gimbal_command(pitch_, yaw_);
            break;
        case 'A':
        case 'a':
            yaw_ += step_;
            RCLCPP_INFO(this->get_logger(), "Поворот: %.1f", yaw_);
            send_gimbal_command(pitch_, yaw_);
            break;
        case 'D':
        case 'd':
            yaw_ -= step_;
            RCLCPP_INFO(this->get_logger(), "Поворот: %.1f", yaw_);
            send_gimbal_command(pitch_, yaw_);
            break;
        case ' ':
            pitch_ = 0.0f;
            yaw_ = 0.0f;
            RCLCPP_INFO(this->get_logger(), "Сброс в центр");
            send_gimbal_command(pitch_, yaw_);
            break;
        case 27:  // ESC
            RCLCPP_INFO(this->get_logger(), "Выход...");
            rclcpp::shutdown();
            break;
        default:
            break;
        }
    }

    // Публикатор в топик /gimbal/commands
    rclcpp::Publisher<mavros_msgs::msg::GimbalManagerSetPitchyaw>::SharedPtr publisher_;

    // Клиент для сервиса MAVROS
    rclcpp::Client<mavros_msgs::srv::GimbalManagerPitchyaw>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct termios old_termios_;

    // Текущие углы камеры
    float pitch_ = 0.0f;
    float yaw_ = 0.0f;
    const float step_ = 2.0f;  // Шаг изменения угла в градусах
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardGimbalController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
