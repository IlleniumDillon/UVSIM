#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class node : public rclcpp::Node
{
public:
    node():Node("set_joint_state")
    {
        this->declare_parameter("joint_names");
        this->declare_parameter("frame_id");
        jointNames = this->get_parameter("joint_names").as_string_array();
        frameId = this->get_parameter("frame_id").as_string();
        pubstate = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("set_joint_trajectory",1);
        auto subCmdCallback = [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) -> void
        {
            trajectory_msgs::msg::JointTrajectory pub;
            pub.joint_names = jointNames;
            pub.header.frame_id=frameId;
            // for(int i = 0; i < msg->data.size(); i++)
            // {
            //     trajectory_msgs::msg::JointTrajectoryPoint p;
            //     p.positions.push_back(msg->data.at(1.5));
            //     pub.points.push_back(p);
            // }
            trajectory_msgs::msg::JointTrajectoryPoint p;
            p.positions.push_back(1.5);
            pub.points.push_back(p);
            pubstate->publish(pub);
        };
        subcmd = this->create_subscription<std_msgs::msg::Float64MultiArray>("set_joint",1,subCmdCallback);
    }
private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pubstate;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subcmd;
    std::vector<std::string> jointNames;
    std::string frameId;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<node>());
    //auto p = std::make_shared<UvPlanner>();
    rclcpp::shutdown();
    return 0;
}