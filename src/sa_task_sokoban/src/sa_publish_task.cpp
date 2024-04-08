#include "Sokoban.hpp"

class socobanTask : public rclcpp::Node
{
public:
    socobanTask() : Node("sokobanTask")
    {
        pub = this->create_publisher<simbridge::msg::ModelState>("/task",1);
        simbridge::msg::ModelState msg;
        geometry_msgs::msg::Pose p;
        p.position.x = 0;
        p.position.y = 0;
        p.position.z = 0;
        std::string name = "obstacle44";
        msg.model_names.push_back(name);
        msg.model_poses.push_back(p);
        pub->publish(msg);
    }
    rclcpp::Publisher<simbridge::msg::ModelState>::SharedPtr pub;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<socobanTask>());
    //auto p = std::make_shared<UvPlanner>();
    rclcpp::shutdown();
    return 0;
}