#pragma once

#include "rclcpp/rclcpp.hpp"
#include "graph_searcher.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav2_map_server/map_io.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <eigen3/Eigen/Eigen>

#include "hungarian_algorithm.hpp"

using namespace Eigen;

class UvAllocator :   public rclcpp::Node
{
public:
    UvAllocator();
    ~UvAllocator(){};
    void subGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
private:
    gridPathFinder pfSolver;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subGoal;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pubGoals;
    int numOfRobots;
    std::vector<std::string> names;
    std::vector<Vector3d> taskList;
    std::vector<int> taskAlloc;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    HungarianAlgorithm allocator;
};