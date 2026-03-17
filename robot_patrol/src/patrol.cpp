#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"

class Patrol : public rclcpp::Node
{
public:
    Patrol() : Node("patrol_node")
    {
        RCLCPP_INFO(this->get_logger(), "Patrol node has been started.");

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", 
        10, 
        std::bind(&Patrol::laser_callback, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float front = msg->ranges[0];
        RCLCPP_INFO(this->get_logger(), "Front distance: %f", front);    
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}