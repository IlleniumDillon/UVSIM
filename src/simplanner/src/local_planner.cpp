#include "local_planner.hpp"

APFPlanner::APFPlanner()
{
}

Vector3d APFPlanner::solve(Vector3d &curPos, Vector3d &curRPY, std::vector<Vector3d> &path)
{
    Vector3d Ftal(0,0,0);
    for(int i = 0; i < laserData.ranges.size(); i++)
    {
        double rotate = curRPY(2) + laserData.angle_min + i*laserData.angle_increment;
        Vector3d obs(
            curPos(0) + cos(rotate)*laserData.ranges.at(i),
            curPos(1) + sin(rotate)*laserData.ranges.at(i),
            curPos(2)
        );
        Ftal += repulsion(obs,curPos,laserData.ranges.at(i),2,0.25);
    }

    double minDis = std::numeric_limits<double>::max();
    int minIndx = -1;
    for(int indx = 0; indx < path.size(); indx++)
    {
        Vector3d P0 = path.at(indx) - curPos;
        double dis = P0.norm();
        if(dis < minDis) 
        {
            minDis = dis;
            minIndx = indx;
        }
    }

    int forward = minIndx+10;
    forward = forward<path.size()?forward:path.size()-1;
    for(int indx = minIndx; indx < forward; indx++)
    {
        Ftal += gravitation(path.at(indx),curPos,3);
    }
    // if(path.size()>0)
    // Ftal += gravitation(path.at(forward),curPos,3);
    Matrix3d rotation_matrix(AngleAxisd(-curRPY(2),Vector3d::UnitZ()));
    Ftal=rotation_matrix*Ftal;
    return Ftal;
}

void APFPlanner::updateLaser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    laserData.angle_increment = msg->angle_increment;
    laserData.angle_max = msg->angle_max;
    laserData.angle_min = msg->angle_min;
    laserData.range_max = msg->range_max;
    laserData.range_min = msg->range_min;
    laserData.ranges = msg->ranges;
    //RCLCPP_INFO(rclcpp::get_logger("apf"),"get laser");
}

Vector3d APFPlanner::gravitation(Vector3d &source, Vector3d &object, double katt)
{
    Vector3d P0 = object - source;
    Vector3d Fatt = -katt * P0;
    return Fatt;
}

Vector3d APFPlanner::repulsion(Vector3d &source, Vector3d &object, float& dis, double krep, double range)
{
    Vector3d Frep(0,0,0);
    if(dis < range)
    {
        Vector3d P0 = object - source;
        double frep_dis = krep*(1/dis - 1/range)*(1/dis/dis)/dis;
        Frep = frep_dis * P0;
    }
    return Frep;
}
