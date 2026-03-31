#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "robot_patrol/srv/get_direction.hpp"

using std::placeholders::_1;

class TestService : public rclcpp::Node {
public:
    TestService() : Node("test_service_node") {

        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",
            10,
            std::bind(&TestService::laser_callback, this, _1)
        );

        client_ = this->create_client<robot_patrol::srv::GetDirection>(
            "/direction_service"
        );

        RCLCPP_INFO(this->get_logger(), "Test Service Node Started");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

        // Wait for service to be available
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for /direction_service...");
            return;
        }

        auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
        request->laser_data = *msg;

        // Async call
auto future = client_->async_send_request(
    request,
    [this](rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {
        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(),
                "Direction: %s", response->direction.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}