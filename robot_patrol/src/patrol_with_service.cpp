#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_patrol/srv/get_direction.hpp"

#include <chrono>

using std::placeholders::_1;

class PatrolWithService : public rclcpp::Node
{
public:
    PatrolWithService() : Node("patrol_with_service_node")
    {
        RCLCPP_INFO(this->get_logger(), "Patrol with Service node started.");

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan",
            10,
            std::bind(&PatrolWithService::laser_callback, this, _1)
        );

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/fastbot_1/cmd_vel",
            10
        );

        client_ = this->create_client<robot_patrol::srv::GetDirection>(
            "/direction_service"
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PatrolWithService::control_loop, this)
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::LaserScan latest_scan_;
    bool scan_received_ = false;

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = *msg;
        scan_received_ = true;
    }

    void control_loop()
    {
        if (!scan_received_) {
            RCLCPP_WARN(this->get_logger(), "No scan received yet");
            return;
        }

        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for /direction_service...");
            return;
        }

        auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
        request->laser_data = latest_scan_;

        auto future = client_->async_send_request(
            request,
            [this](rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {

                try {
                    auto response = future.get();

                    geometry_msgs::msg::Twist cmd;

                    if (response->direction == "forward") {
                        cmd.linear.x = 0.1;
                        cmd.angular.z = 0.0;
                    }
                    else if (response->direction == "left") {
                        cmd.linear.x = 0.1;
                        cmd.angular.z = 0.5;
                    }
                    else if (response->direction == "right") {
                        cmd.linear.x = 0.1;
                        cmd.angular.z = -0.5;
                    }

                    RCLCPP_INFO(this->get_logger(),
                        "Direction: %s", response->direction.c_str());

                    cmd_pub_->publish(cmd);

                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed");
                }
            }
        );
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolWithService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}