#pragma once

#include "graph_searcher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//#include "uvinterfaces/srv/uv_pathplan.hpp"
//#include "uvinterfaces/msg/uv_map.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using namespace Eigen;
class UvPlanner :   public rclcpp::Node
{
public:
    UvPlanner();
    //void subMapCallback(const uvinterfaces::msg::UvMap::SharedPtr msg);
    /*void srvSolveCallback(const uvinterfaces::srv::UvPathplan::Request::SharedPtr req,
                          const uvinterfaces::srv::UvPathplan::Response::SharedPtr res);*/
    void subGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void subMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void localPlanner();
public:
    //UV::AStar solver;
    // UV::JPS solver;
    gridPathFinder solver;
    Vector3d lastGoal;
    Vector3d CurrPos;
    Vector3d CurrRPY;
    std::vector<Vector3d> path;
    int currentPathIndx;
    ///TODO:
    ///sub for map
    //rclcpp::Subscription<uvinterfaces::msg::UvMap>::SharedPtr subMap;
    ///srv for plan
    //rclcpp::Service<uvinterfaces::srv::UvPathplan>::SharedPtr srvPath;
    ///
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subGoal;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subnMap;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmd;
    rclcpp::TimerBase::SharedPtr timer;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    bool readyPlan = false;
    bool inProgress = false;
};
