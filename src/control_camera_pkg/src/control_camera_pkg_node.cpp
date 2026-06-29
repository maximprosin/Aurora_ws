#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ControlCamera: public rclcpp::Node {
public:
    ControlCamera() : Node("control_camera_node") {



    }

private:



};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlCamera>());
    rclcpp::shutdown();

    return 0;
}
