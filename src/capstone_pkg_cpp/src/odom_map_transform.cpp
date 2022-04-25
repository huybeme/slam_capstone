#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <string>

class odom_map_transform : public rclcpp::Node
{
public:
    odom_map_transform() : Node("odom_map_transform")
    {
        timer = this->create_wall_timer(
            std::chrono::milliseconds(1000), std::bind(&odom_map_transform::on_timer, this)
        );
    }
 
private:
    void on_timer()
    {
        RCLCPP_INFO(this->get_logger(), "node node node");
    }

    // end of private functions - variables here
    rclcpp::TimerBase::SharedPtr timer;

};
     
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<odom_map_transform>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}