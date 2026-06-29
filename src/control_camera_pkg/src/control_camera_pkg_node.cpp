#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class ControlCamera: public rclcpp::Node {
public:
    ControlCamera() : Node("control_camera_node") {

        subscriber = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_body",
            10,  // Глубина очереди (или используйте rclcpp::QoS)
            std::bind(&ControlCamera::control_camera_callback, this, std::placeholders::_1));

        //Иницилизация публикатора
        publisher =
            this->create_publisher<geometry_msgs::msg::TwistStamped>("processed_image",
                                                            10);


    }

private:


    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr   publisher;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber;


    geometry_msgs::msg::TwistStamped liner;

    void control_camera_callback(const geometry_msgs::msg::TwistStamped msg){




    }


};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlCamera>());
    rclcpp::shutdown();

    return 0;
}
