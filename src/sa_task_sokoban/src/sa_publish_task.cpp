#include "Sokoban.hpp"
#include <termios.h>
#include <iostream>
#include <optional>

std::optional<std::string> noblock_input(std::istream& in = std::cin)
{
    in.sync_with_stdio(false);

    if (in.rdbuf()->in_avail() > 0) {
        char buffer[1024];

        in.getline(buffer, sizeof(buffer));

        return buffer;
    }

    return {};
}
std::vector<string> model_names_A = {
        "00","03","02","01","14","25","36"
};
std::vector<Vector2d> model_poses_A = {
        {3.9,2.7},{3.3,3.3},{3.3,2.1},{2.7,3.3},{2.7,2.1},{2.1,3.3},{2.1,2.1}
};
std::vector<string> model_names_I = {
        "09","10","13","17","18"
};
std::vector<Vector2d> model_poses_I = {
        {3.9,-2.1},{2.1,-2.1},{0.9,-3.9},{0.9,-2.7},{0.9,-3.3}
};
std::vector<string> model_names_U = {
        "20","19","21","28","27","29","23","26"
};
std::vector<Vector2d> model_poses_U = {
        {-0.3,3.3},{-0.9,3.3},{-1.5,3.3},{-0.3,2.1},{-0.9,2.1},{-1.5,2.1},{-3.3,3.9},{-3.3,3.3}
};
std::vector<string> model_names_S = {
        "42","41","40","39","38","37","35"
};
std::vector<Vector2d> model_poses_S = {
        {-0.3,-2.1},{-0.3,-1.5},{-0.9,-0.9},{-1.5,-2.1},{-1.5,-1.5},{-2.1,-0.9},{-2.1,-1.5}
};

std::vector<std::vector<string>> model_names = {
        model_names_A,model_names_I,model_names_U,model_names_S
};
std::vector<std::vector<Vector2d>> model_poses = {
        model_poses_A,model_poses_I,model_poses_U,model_poses_S
};

class socobanTask : public rclcpp::Node
{
public:
    socobanTask() : Node("sokobanTask")
    {
        pub = this->create_publisher<simbridge::msg::ModelState>("/task",1);
        // simbridge::msg::ModelState msg;

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

        // geometry_msgs::msg::Pose p;
        // p.position.x = 0.9;
        // p.position.y = 0.3;
        // p.position.z = 0;
        // std::string name = "obstacle36";
        // msg.model_names.push_back(name);
        // msg.model_poses.push_back(p);
        // p.position.x = 0.3;
        // p.position.y = 0.9;
        // name = "obstacle43";
        // msg.model_names.push_back(name);
        // msg.model_poses.push_back(p);
        // pub->publish(msg);
    }
    rclcpp::Publisher<simbridge::msg::ModelState>::SharedPtr pub;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    auto nh = std::make_shared<socobanTask>();
    RCLCPP_INFO(nh->get_logger(), "INPUT A I U S TO PUBLISH TASK");
    while (rclcpp::ok())
    {
        rclcpp::spin_some(nh);
        auto input = noblock_input();
        if (input.has_value())
        {
            string str = input.value();
            int indx = -1;
            if (str[0] == 'a')
            {
                indx = 0;
            }
            else if (str[0] == 'i')
            {
                indx = 1;
            }
            else if (str[0] == 'u')
            {
                indx = 2;
            }
            else if (str[0] == 's')
            {
                indx = 3;
            }
            if (indx != -1)
            {
                simbridge::msg::ModelState msg;
                for (int i = 0; i < model_names[indx].size(); i++)
                {
                    string name = "obstacle" + model_names[indx][i];
                    msg.model_names.push_back(name);
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = model_poses[indx][i](0);
                    pose.position.y = model_poses[indx][i](1);
                    pose.position.z = 0;
                    msg.model_poses.push_back(pose);
                }
                nh->pub->publish(msg);
                RCLCPP_INFO(nh->get_logger(), "PUBLISH TASK %c",str[0]);
            }
            else
            {
                RCLCPP_INFO(nh->get_logger(), "INPUT A I U S TO PUBLISH TASK");
            }
        }
    }
    rclcpp::shutdown();
    return 0;
}