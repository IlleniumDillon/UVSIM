#include "rclcpp/rclcpp.hpp"
#include "UvPlanner.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<UvPlanner>());
    //auto p = std::make_shared<UvPlanner>();
    rclcpp::shutdown();
    return 0;
}