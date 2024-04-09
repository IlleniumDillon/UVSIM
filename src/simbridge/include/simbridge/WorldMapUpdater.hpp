#pragma once

#include "rclcpp/rclcpp.hpp"
#include "simbridge/msg/model_state.hpp"
#include "simbridge/msg/model_ignore.hpp"
#include "simbridge/srv/query_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "opencv2/opencv.hpp"

class WorldMapUpdater : public rclcpp::Node
{
public:
    ///@brief 前四个参数是指地图边界的参数
    WorldMapUpdater(double width, double hight, double wallthickness, double ori_x, double ori_y, double res = 0.05);
    void modelStateCallback(const simbridge::msg::ModelState::SharedPtr msg);
    void queryMapServace(const simbridge::srv::QueryMap_Request::SharedPtr request,
        simbridge::srv::QueryMap_Response::SharedPtr responce);
private:
    rclcpp::Subscription<simbridge::msg::ModelState>::SharedPtr sub_model;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_ori;
    rclcpp::Service<simbridge::srv::QueryMap>::SharedPtr srv_map;
    double resolution = 0.05;
    double origin_x,origin_y,origin_z;
    cv::Mat MapBlank;
    std::vector<std::string> ignoreNames;
    simbridge::msg::ModelState modelStates;
};