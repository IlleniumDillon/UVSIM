#include "UvAllocator.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<UvAllocator>());
    //auto p = std::make_shared<UvPlanner>();
    rclcpp::shutdown();
    return 0;
}