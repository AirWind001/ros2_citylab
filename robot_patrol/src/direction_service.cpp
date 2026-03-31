#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "rclcpp/logging.hpp"
#include "robot_patrol/srv/get_direction.hpp"

#include <cmath>
#include <limits>

using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {
public:
    DirectionService() : Node("direction_service_node") {

        service_ = this->create_service<robot_patrol::srv::GetDirection>(
            "/direction_service",
            std::bind(&DirectionService::handle_request, this, _1, _2)
        );

        RCLCPP_INFO(this->get_logger(), "Service Server Ready");
    }

private:
    rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;

    void handle_request(
        const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
        std::shared_ptr<robot_patrol::srv::GetDirection::Response> response)
    {
        
        RCLCPP_INFO(this->get_logger(), "Service Requested");

        auto msg = request->laser_data;

        double total_right = 0.0;
        double total_front = 0.0;
        double total_left  = 0.0;

        int count_right = 0, count_front = 0, count_left = 0;

        int center_index = msg.ranges.size() / 2;
        double front_dist = msg.ranges[center_index];

        // Loop through only front 180°
        for (size_t i = 0; i < msg.ranges.size(); i++) {

            double angle = msg.angle_min + i * msg.angle_increment;
        RCLCPP_DEBUG(this->get_logger(),
            "angle BEFORE normalise: %f",
            angle);         
        // Normalize to [-π, π]
        angle = atan2(sin(angle), cos(angle));
        RCLCPP_DEBUG(this->get_logger(),
            "angle after normalise: %f",
            angle); 
            // Only consider [-pi/2, pi/2]
            if (angle > M_PI/2 && angle < 3*M_PI/2)
                continue;
            double dist = msg.ranges[i];

            if (std::isinf(dist))
                continue;
            // RIGHT: [-π/2, -π/6)
            if (angle >= -M_PI/2 && angle < -M_PI/6) {
                total_right += dist;
                count_right++;
                RCLCPP_DEBUG(this->get_logger(),
                    "Inside Right, total_right : %.2f count_right: %d",
                    total_right, count_right);      
            }

            // FRONT: [-π/6, π/6)
            else if (angle >= -M_PI/6 && angle < M_PI/6) {
                total_front += dist;
                count_front++;
                RCLCPP_DEBUG(this->get_logger(),
                    "Inside Front, total_front : %.2f count_front: %d",
                    total_front, count_front);    
            }            
            // LEFT: [30, 90)
            else if (angle >= M_PI/6 && angle <= M_PI/2) {
                total_left += dist;
                count_left++;
                RCLCPP_DEBUG(this->get_logger(),
                    "Inside Left, total_left : %.2f count_left: %d",
                    total_left, count_left);                
            }
        }

        RCLCPP_DEBUG(this->get_logger(),
            "Totals → Right: %.2f Front: %.2f Left: %.2f",
            total_right, total_front, total_left);

        // Decision logic
        if (front_dist > 0.35) {
            response->direction = "forward";
            RCLCPP_DEBUG(this->get_logger(), "Decision: FORWARD");
            return;
        }
        else{
        // Choose max section
        if (total_left > total_right && total_left > total_front) {
            response->direction = "left";
        }
        else if (total_right > total_left && total_right > total_front) {
            response->direction = "right";
        }
        else {
            response->direction = "forward";
        }        
        }


        RCLCPP_DEBUG(this->get_logger(),
            "Decision: %s", response->direction.c_str());

        RCLCPP_INFO(this->get_logger(), "Service Completed");

    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectionService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}