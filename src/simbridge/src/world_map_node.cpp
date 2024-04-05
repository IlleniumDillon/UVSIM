#include "WorldMapUpdater.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<WorldMapUpdater>(8.7,11.7,0.15,-0.3,0,0.05));
    //auto p = std::make_shared<UvPlanner>();
    rclcpp::shutdown();
    return 0;
}