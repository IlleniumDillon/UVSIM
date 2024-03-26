#include "SamplePlanner.hpp"
// #include "matplotlibcpp.h"
using namespace Eigen;

UvPlannerSample::UvPlannerSample():Node("uvplanner_node")
{
    lastGoal = Vector3d(0,0,0);
    subnMap = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",1,
        std::bind(&UvPlannerSample::subMapCallback,this,std::placeholders::_1));
    /*srvPath = this->create_service<uvinterfaces::srv::UvPathplan>("path",
        std::bind(&UvPlanner::srvSolveCallback,this,std::placeholders::_1,std::placeholders::_2));*/
    subGoal = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",1,
        std::bind(&UvPlannerSample::subGoalCallback,this,std::placeholders::_1));
    pubPath = this->create_publisher<nav_msgs::msg::Path>("/path",1);
    readyPlan = true;
}

void UvPlannerSample::subMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if(inProgress)
    {
        RCLCPP_ERROR(this->get_logger(),"cannot update map.");
        return;
    }
    Vector3d ol(msg->info.origin.position.x,
        msg->info.origin.position.y,
        msg->info.origin.position.z);

    solver.SetMap(msg->info.resolution,msg->info.width,msg->info.height,1,ol);
    //uint8_t* dmap = new uint8_t[msg->info.height*msg->info.width];
    RCLCPP_INFO(this->get_logger(),"%d,%d",msg->info.height,msg->info.width);
    if(solver.mapData!=nullptr)
    {
        delete solver.mapData;
    }
    solver.mapData = new uint8_t[msg->info.height * msg->info.width]{0};
    for(int i = 0; i < msg->info.height; i++)
    {
        for(int j = 0; j < msg->info.width; j++)
        {
            //RCLCPP_INFO(this->get_logger(),"%d",msg->data.at(i*msg->info.width+j));
            if(msg->data.at(i*msg->info.width+j) == -1||
                msg->data.at(i*msg->info.width+j) == 100)
            {
                solver.mapData[j*solver.mapY+i] = 1;
                // RCLCPP_INFO(this->get_logger(),"%d,%d",i,j);
            }
            
        }
    }
    // cv::Mat testM = cv::Mat(solver.mapX,solver.mapY,CV_8UC1,solver.mapData);
    // testM*=255;
    // cv::imshow("img",testM);
    // cv::waitKey(0);

    readyPlan = true;
}
void UvPlannerSample::subGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if(!readyPlan)
    {
        RCLCPP_ERROR(this->get_logger(),"cannot plan without map.");
        return;
    }
    inProgress = true;

    nav_msgs::msg::Path req;
    req.header.frame_id="/map";
    req.header.stamp = this->now();
    Vector3d goal(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    RCLCPP_INFO(this->get_logger(),"[%f,%f,%f]->[%f,%f,%f]",lastGoal.x(),lastGoal.y(),lastGoal.z(),goal.x(),goal.y(),goal.z());
    solver.ResetRRT(solver.rviz2map(lastGoal),solver.rviz2map(goal));
     
    RCLCPP_INFO(this->get_logger(),"reset");
    RCLCPP_INFO(this->get_logger(),"goal: %f, %f",solver.goal_data.x()* solver.inv_resolution,solver.goal_data.y()* solver.inv_resolution);
    // RCLCPP_INFO(this->get_logger(),"indx:%d",(int)floor(solver.goal_data.x())*solver.mapY + (int)floor(solver.goal_data.y()));
    if(solver.mapData[(int)floor(solver.goal_data.x()* solver.inv_resolution)*solver.mapY + (int)floor(solver.goal_data.y()* solver.inv_resolution)]==0)
    {
        RCLCPP_INFO(this->get_logger(),"0");
    }



    auto start = std::chrono::system_clock::now();
    auto path = solver.Planning();

    RCLCPP_INFO(this->get_logger(),"planning");

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end - start;
    RCLCPP_INFO(this->get_logger(),"cost time:%f",diff);

    req.poses.clear();
    
    for(int i = path.size()-1; i >=0; i--)
    {
        geometry_msgs::msg::PoseStamped temp;
        temp.pose.position.x = path.at(i).x();
        temp.pose.position.y = path.at(i).y();
        temp.pose.position.z = path.at(i).z();
        req.poses.push_back(temp);
    }

    lastGoal = goal;
    pubPath->publish(req);
    inProgress = false;
}

