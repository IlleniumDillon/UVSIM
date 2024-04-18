#include "Sokoban.hpp"
std::vector<string> model_names = {
        "obstacle36", "obstacle43", "obstacle18", "obstacle05", "obstacle34", "obstacle16", "obstacle33", "obstacle17"
};
std::vector<Vector3d> model_poses = {
    {0.9,0.3,0},
    {0.3,0.9,0},
    {0.3,-0.3,0},
    {-0.3,0.9,0},
    {-0.3,0.3,0},
    {-0.3,-0.3,0},
    {-0.9,0.9,0},
    {-0.9,-0.3,0}
};
class socobanTask : public rclcpp::Node
{
public:
    socobanTask() : Node("sokobanTask")
    {
        pub = this->create_publisher<simbridge::msg::ModelState>("/task",1);
        simbridge::msg::ModelState msg;
        // for (int i = 0; i < model_names.size(); i++)
        // {
        //     string name = model_names[i];
        //     msg.model_names.push_back(name);
        //     geometry_msgs::msg::Pose pose;
        //     pose.position.x = model_poses[i](0);
        //     pose.position.y = model_poses[i](1);
        //     pose.position.z = model_poses[i](2);
        //     msg.model_poses.push_back(pose);
        // }
        geometry_msgs::msg::Pose p;
        p.position.x = 0.9;
        p.position.y = 0.3;
        p.position.z = 0;
        std::string name = "obstacle36";
        msg.model_names.push_back(name);
        msg.model_poses.push_back(p);
        p.position.x = 0.3;
        p.position.y = 0.9;
        name = "obstacle43";
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