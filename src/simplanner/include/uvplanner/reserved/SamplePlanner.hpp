#pragma once

#include "rclcpp/rclcpp.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rrt.h"

using namespace Eigen;
class UvPlannerSample :   public rclcpp::Node
{
public:
    UvPlannerSample();

    void subGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void subMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
public:
    UV::RRT solver;
    Vector3d lastGoal;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subGoal;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subnMap;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

    bool readyPlan = false;
    bool inProgress = false;
};