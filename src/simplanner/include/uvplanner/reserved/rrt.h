#pragma once

// #include <Eigen/Eigen>
// #include <Eigen/StdVector>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include "raycast.h"
#include <random>
#include "rclcpp/rclcpp.hpp"
using namespace Eigen;
using namespace std;
namespace UV
{


// class Vector3d
// {
// public:
//     Vector3d(double x_, double y_,double z_):x(x),y(y),z(z){}
//     double x = 0;
//     double y = 0;
//     double z = 0;
// };

class RRT_Node 
{
public:
    RRT_Node* fatherPtr = nullptr;
    Vector3d data;

    RRT_Node(Vector3d data_, RRT_Node* father_ptr = nullptr):fatherPtr(father_ptr),data(data_){}
//是否将Info变为private，使用eigen
};

class RRT
{
public:
    Vector3d start_data;
    Vector3d goal_data;
    std::vector<RRT_Node*> node_list;
    double step_size = 0.3;
    
    //下右上xyz
    Vector3d origin;
    int mapX=10,mapY=10,mapZ=1;
    double resolution;
    double inv_resolution;
    double min_double = 0.0000001;
    //xyz顺序
    uint8_t* mapData=nullptr;


    int goal_sample_rate = 5;

    std::mt19937 goal_gen;
    std::uniform_int_distribution<int> goal_dis;
    std::uniform_real_distribution<double> x_dis;
    std::uniform_real_distribution<double> y_dis;
    std::uniform_real_distribution<double> z_dis;




    // RRT(double step_size_ = 0.5, int goal_sample_rate_ = 5)  // 障碍物，步长（节点之间的距离），目标采样率（有多少概率直接采样到目标点）
    //   : step_size(step_size_),
    //     goal_sample_rate(goal_sample_rate_),
    //     goal_gen(std::random_device{}()),
    //     goal_dis(std::uniform_int_distribution<int>(0, 100)),
    //     x_dis(std::uniform_real_distribution<double>(0, mapX*resolution)),
    //     y_dis(std::uniform_real_distribution<double>(0, mapY*resolution)),
    //     z_dis(std::uniform_real_distribution<double>(0, mapZ*resolution))      
    //     {
    //         // RRT_Node* start_node = new RRT_Node(start_data);
    //         // node_list.push_back(start_node);
    //     }

    void SetMap(double _resolution, int max_x_id, int max_y_id, int max_z_id,Vector3d _origin);
    void ResetRRT(Vector3d start, Vector3d goal);
    RRT_Node* Sample();
    RRT_Node* Near(RRT_Node* x_rand);
    RRT_Node* Step(RRT_Node* x_rand, RRT_Node* x_near);
    bool CollisionFree( RRT_Node* x_near,RRT_Node* x_new);
    void AddNode(RRT_Node* x_new, RRT_Node* x_father);
    bool SuccessCheck(RRT_Node* x_new);

    std::vector<Vector3d> Planning();

    Vector3d map2rviz(const Vector3d& index) const;

    Vector3d rviz2map(const Vector3d& pt) const;

    double res_distance(double p,double direction);
    void fixInMap(Eigen::Vector3d& new_node);

};



}

