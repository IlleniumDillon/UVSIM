#pragma once

#include "rclcpp/rclcpp.hpp"
#include "simbridge/msg/model_state.hpp"
#include "simbridge/msg/model_ignore.hpp"
#include "simbridge/msg/add_joint.hpp"
#include "simbridge/srv/query_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "graph_searcher.hpp"
#include "hungarian_algorithm.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "sokoban_solver.hpp"

#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

class APFSolverPrivate;
class taskExecutorPrivate;

class Sokoban : public rclcpp::Node
{
    enum ExecState
    {
        BeforeInit,
        Idle,
        Shift,
        Activate,
        ShiftWithBox,
        Deactivate,
    };
public:
    Sokoban();
    void timerCallback();
private:
    void updatePose();

    rclcpp::TimerBase::SharedPtr timer;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_arm_arm_joint;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_arm_hand_joint;
    //rclcpp::Publisher<simbridge::msg::ModelIgnore>::SharedPtr pub_model_ignore;
    rclcpp::Publisher<simbridge::msg::AddJoint>::SharedPtr pub_addjoint;

    rclcpp::Subscription<simbridge::msg::ModelState>::SharedPtr sub_model_state;
    rclcpp::Subscription<simbridge::msg::ModelState>::SharedPtr sub_task;
    //rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_dilate;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;

    //rclcpp::Client<simbridge::srv::QueryMap>::SharedPtr cli_map;

    std::shared_ptr<APFSolverPrivate> impl_apfsolver;
    std::shared_ptr<taskExecutorPrivate> impl_taskexecutor;

    ExecState curState = BeforeInit;
    ExecState lastState = BeforeInit;

    std::string robotName;
    std::string armName;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::Quaternion q;
    tf2::Quaternion quat;
    std::string fromFrameRel;
    std::string toFrameRel;

    Vector3d CurrPos;
    Vector3d CurrRPY;
};