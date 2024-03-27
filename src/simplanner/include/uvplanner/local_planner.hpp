#pragma once

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/msg/laser_scan.hpp>
using namespace Eigen;
/// TODO:this class is for 2D, fix something to compatible 3D
class APFPlanner
{
public:
    APFPlanner();
    ~APFPlanner(){};

    Vector3d solve(Vector3d& curPos, Vector3d& curRPY, std::vector<Vector3d>& path);
    void updateLaser(const sensor_msgs::msg::LaserScan::SharedPtr msg);

private:
    Vector3d gravitation(Vector3d& source, Vector3d& object, double katt);
    Vector3d repulsion(Vector3d &source, Vector3d &object, float& dis, double krep, double range);
    sensor_msgs::msg::LaserScan laserData;
};