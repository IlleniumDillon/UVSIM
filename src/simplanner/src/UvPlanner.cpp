#include "UvPlanner.hpp"
using namespace Eigen;
UvPlanner::UvPlanner():Node("uvplanner_node")
{
    lastGoal = Vector3d(0,0,0);
    CurrPos = Vector3d(0,0,0);
    CurrRPY = Vector3d(0,0,0);
    currentPathIndx = 0;
    subnMap = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",1,
        std::bind(&UvPlanner::subMapCallback,this,std::placeholders::_1));
    /*srvPath = this->create_service<uvinterfaces::srv::UvPathplan>("path",
        std::bind(&UvPlanner::srvSolveCallback,this,std::placeholders::_1,std::placeholders::_2));*/
    subGoal = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",1,
        std::bind(&UvPlanner::subGoalCallback,this,std::placeholders::_1));
    subScan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",1,
        std::bind(&APFPlanner::updateLaser,&apfPlanner,std::placeholders::_1));
    pubPath = this->create_publisher<nav_msgs::msg::Path>("/path",1);
    pubCmd = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1);
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    timer = this->create_wall_timer(100ms, std::bind(&UvPlanner::localPlanner, this));
    readyPlan = false;
}

void UvPlanner::subMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if(inProgress)
    {
        RCLCPP_ERROR(this->get_logger(),"cannot update map.");
        return;
    }
    Vector3d ol(msg->info.origin.position.x,
        msg->info.origin.position.y,
        msg->info.origin.position.z);
    Vector3d ou(msg->info.origin.position.x,
        msg->info.origin.position.y,
        msg->info.origin.position.z+msg->info.resolution);
    solver.initGridMap(msg->info.resolution,ol,ou,msg->info.width,msg->info.height,1);
    //uint8_t* dmap = new uint8_t[msg->info.height*msg->info.width];
    RCLCPP_INFO(this->get_logger(),"%d,%d",msg->info.height,msg->info.width);
    for(int i = 0; i < msg->info.height; i++)
    {
        for(int j = 0; j < msg->info.width; j++)
        {
            //RCLCPP_INFO(this->get_logger(),"%d",msg->data.at(i*msg->info.width+j));
            if(msg->data.at(i*msg->info.width+j) == -1||
                msg->data.at(i*msg->info.width+j) == 100)
            {
                solver.setObs(j,i,0);
            }
            
        }
    }
    readyPlan = true;
}
void UvPlanner::localPlanner()
{
    double inv_timesample = 10;
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::Quaternion q;
    tf2::Quaternion quat;
    std::string fromFrameRel = "base_link";
    std::string toFrameRel = "map";
    try 
    {
        t = tf_buffer->lookupTransform(toFrameRel, fromFrameRel,tf2::TimePointZero);
        q = t.transform.rotation;
        tf2::convert(q, quat);
        tf2::Matrix3x3 m(quat);
        CurrPos(0) = t.transform.translation.x;
        CurrPos(1) = t.transform.translation.y;
        CurrPos(2) = t.transform.translation.z;
        m.getRPY(CurrRPY(0),CurrRPY(1),CurrRPY(2));
        // RCLCPP_INFO(this->get_logger(), "Current position:[%f,%f,%f]",CurrPos(0),CurrPos(1),CurrPos(2));
        // RCLCPP_INFO(this->get_logger(), "Current rotation:[%f,%f,%f]",CurrRPY(0),CurrRPY(1),CurrRPY(2));
    } 
    catch (const tf2::TransformException & ex) 
    {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
    }

    geometry_msgs::msg::Twist cmd;
    Vector3d f = apfPlanner.solve(CurrPos,CurrRPY,path);
    RCLCPP_INFO(this->get_logger(),"F[%f,%f,%f]",f(0),f(1),f(2));
    double fx = f(0),fy = f(1);
    if(fx > 1) fx = 0.5;
    if(fx < -1) fx = -0.5;
    if(fy > 1) fy = 0.5;
    if(fy < -1) fy = -0.5;

    cmd.linear.x = fx;
    cmd.angular.z = fy;
    
    pubCmd->publish(cmd);
    // geometry_msgs::msg::Twist cmd;
    // if(currentPathIndx >= path.size())
    // {
    //     cmd.linear.x = 0;
    //     cmd.linear.y = 0;
    //     cmd.linear.z = 0;
    //     pubCmd->publish(cmd);
    //     return;
    // }
    
    // Vector3d deltap = path.at(currentPathIndx) - CurrPos;
    // double dx = deltap(0);
    // double dy = deltap(1);
    // double distance = sqrt(dx*dx+dy*dy);
    // if(distance < 0.05) currentPathIndx++;
    // if(currentPathIndx >= path.size())
    // {
    //     cmd.linear.x = 0;
    //     cmd.linear.y = 0;
    //     cmd.linear.z = 0;
    //     pubCmd->publish(cmd);
    //     return;
    // }
    // deltap = path.at(currentPathIndx) - CurrPos;
    // dx = deltap(0);
    // dy = deltap(1);
    // distance = sqrt(dx*dx+dy*dy);
    // double rotate = atan2(dy,dx) - CurrRPY(2);
    // if(abs(rotate) > 0.15)
    // {
    //     cmd.linear.x = 0;
    //     cmd.angular.z = rotate > 0 ? 1 : 1;
    // }
    // else
    // {
    //     cmd.angular.z = 0;
    //     cmd.linear.x = 0.5;
    // }
    // pubCmd->publish(cmd);
}
void UvPlanner::subGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if(!readyPlan)
    {
        RCLCPP_ERROR(this->get_logger(),"cannot plan without map.");
        return;
    }
    inProgress = true;
    solver.resetUsedGrids();
    nav_msgs::msg::Path req;
    req.header.frame_id="/map";
    req.header.stamp = this->now();
    Vector3d goal(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    RCLCPP_INFO(this->get_logger(),"[%f,%f,%f]->[%f,%f,%f]",CurrPos.x(),CurrPos.y(),CurrPos.z(),goal.x(),goal.y(),goal.z());
    auto start = std::chrono::system_clock::now();
    solver.graphSearch(CurrPos,goal,false);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end - start;
    RCLCPP_INFO(this->get_logger(),"cost time:%f",diff);
    // if(res)
    // {
        req.poses.clear();
        path.clear();
        path = solver.getPath();
        currentPathIndx = 0;
        for(int i = 0; i < path.size(); i++)
        {
            geometry_msgs::msg::PoseStamped temp;
            temp.pose.position.x = path.at(i).x();
            temp.pose.position.y = path.at(i).y();
            temp.pose.position.z = path.at(i).z();
            req.poses.push_back(temp);
        }
    // }
    // else
    // {
    //     req.poses.clear();
    //     RCLCPP_ERROR(this->get_logger(),"no valid plan.");
    // }
    lastGoal = goal;
    pubPath->publish(req);
    inProgress = false;
}
