#include "rclcpp/rclcpp.hpp"

class Patrol : public rclcpp::Node
{
public:
    Patrol() : Node("patrol_node")
    {
    }

private:

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}