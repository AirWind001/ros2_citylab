#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "robot_patrol/action/go_to_pose.hpp"

#include <cmath>

using namespace std::placeholders;

class GoToPose : public rclcpp::Node
{
public:
    using GoToPoseAction = robot_patrol::action::GoToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

    GoToPose() : Node("go_to_pose_action_server")
    {
        // Action server
        action_server_ = rclcpp_action::create_server<GoToPoseAction>(
            this,
            "/go_to_pose",
            std::bind(&GoToPose::handle_goal, this, _1, _2),
            std::bind(&GoToPose::handle_cancel, this, _1),
            std::bind(&GoToPose::handle_accepted, this, _1)
        );

        // Subscriber
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&GoToPose::odom_callback, this, _1)
        );

        // Publisher
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "GoToPose Action Server Started");
    }

private:
    rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    geometry_msgs::msg::Pose2D current_pos_;
    geometry_msgs::msg::Pose2D desired_pos_;

    // ---------------------------
    // Quaternion → Yaw
    // ---------------------------
    double get_yaw_from_quaternion(double x, double y, double z, double w)
    {
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    // ---------------------------
    // ODOM CALLBACK
    // ---------------------------
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pos_.x = msg->pose.pose.position.x;
        current_pos_.y = msg->pose.pose.position.y;

        auto q = msg->pose.pose.orientation;
        current_pos_.theta = get_yaw_from_quaternion(q.x, q.y, q.z, q.w);
    }

    // ---------------------------
    // ACTION HANDLERS
    // ---------------------------
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const GoToPoseAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal");
        desired_pos_ = goal->goal_pos;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle>)
    {
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread{std::bind(&GoToPose::execute, this, goal_handle)}.detach();
    }

    // ---------------------------
    // CONTROL LOOP
    // ---------------------------
    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        auto feedback = std::make_shared<GoToPoseAction::Feedback>();
        auto result = std::make_shared<GoToPoseAction::Result>();

        rclcpp::Rate rate(10);

        while (rclcpp::ok())
        {
            // Distance to goal
            double dx = desired_pos_.x - current_pos_.x;
            double dy = desired_pos_.y - current_pos_.y;

            double distance = std::sqrt(dx * dx + dy * dy);

            // Stop condition
            if (distance < 0.1)
            {
                geometry_msgs::msg::Twist stop;
                cmd_pub_->publish(stop);

                result->status = true;
                goal_handle->succeed(result);

                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                return;
            }

            // Desired angle
            double target_theta = std::atan2(dy, dx);
            double error_theta = target_theta - current_pos_.theta;

            // Normalize angle
            while (error_theta > M_PI) error_theta -= 2 * M_PI;
            while (error_theta < -M_PI) error_theta += 2 * M_PI;

            geometry_msgs::msg::Twist cmd;

            cmd.linear.x = 0.2;
            cmd.angular.z = 1.0 * error_theta;

            cmd_pub_->publish(cmd);

            // Feedback
            feedback->current_pos = current_pos_;
            goal_handle->publish_feedback(feedback);

            rate.sleep();
        }
    }
};

// ---------------------------
// MAIN
// ---------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToPose>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}