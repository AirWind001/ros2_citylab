#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include <chrono>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

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

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel",
        10
        );

        // loop of 10Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Patrol::control_loop, this)
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double direction_ = 0.0;

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    int start_index = std::ceil(( -M_PI/2 - msg->angle_min) / msg->angle_increment);
    int end_index   = std::floor((  M_PI/2 - msg->angle_min) / msg->angle_increment);

    double max_distance = 0.0;
    int best_index = start_index;

    for (int i = start_index; i <= end_index; i++) {

        double dist = msg->ranges[i];

        if (std::isinf(dist)) continue;

        if (dist > max_distance) {
            max_distance = dist;
            best_index = i;
        }
    }

    // Convert index → angle
    double best_angle = msg->angle_min + best_index * msg->angle_increment;

    // Check obstacle in front
    int center_index = msg->ranges.size() / 2;
    double front_dist = msg->ranges[center_index];

    if (front_dist < 0.45) {        // increase for real robot due to safety considerations
        direction_ = best_angle;
    } else {
        direction_ = 0.0;
    }
}
void control_loop() {

    geometry_msgs::msg::Twist cmd;

    cmd.linear.x = 0.05;            // Decrease linear velocity due to safety considerations
    cmd.angular.z = direction_ / 2.0;

    cmd_pub_->publish(cmd);
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