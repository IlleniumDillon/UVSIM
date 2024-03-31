#include "UvAllocator.hpp"

UvAllocator::UvAllocator() :Node("uvallocator_node")
{
    this->declare_parameter("robot_list");
    this->declare_parameter("map_file", "");
    std::string map_file = this->get_parameter("map_file").as_string();
    names = this->get_parameter("robot_list").as_string_array();
    numOfRobots = names.size();
    nav_msgs::msg::OccupancyGrid map;
    nav2_map_server::loadMapFromYaml(map_file,map);
    Vector3d ol(map.info.origin.position.x,
        map.info.origin.position.y,
        map.info.origin.position.z);
    Vector3d ou(map.info.origin.position.x,
        map.info.origin.position.y,
        map.info.origin.position.z+map.info.resolution);
    pfSolver.initGridMap(map.info.resolution,ol,ou,map.info.width,map.info.height,1);
    //uint8_t* dmap = new uint8_t[map.info.height*map.info.width];
    RCLCPP_INFO(this->get_logger(),"%d,%d",map.info.height,map.info.width);
    for(int i = 0; i < map.info.height; i++)
    {
        for(int j = 0; j < map.info.width; j++)
        {
            //RCLCPP_INFO(this->get_logger(),"%d",map.data.at(i*map.info.width+j));
            if(map.data.at(i*map.info.width+j) == -1||
                map.data.at(i*map.info.width+j) == 100)
            {
                pfSolver.setObs(j,i,0);
            }
        }
    }

    subGoal = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose",1,
        std::bind(&UvAllocator::subGoalCallback,this,std::placeholders::_1));

    for(int i = 0; i < numOfRobots; i++)
    {
        pubGoals.push_back(
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/"+names.at(i)+"/goal_pose",1
            )
        );
    }

    taskList.clear();
    taskAlloc.clear();

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
}

void UvAllocator::subGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    Vector3d taskPoint(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    taskList.push_back(taskPoint);
    RCLCPP_INFO(this->get_logger(),"get task ppoint (%d/%d):[%f,%f,%f]",
        taskList.size(),numOfRobots,taskPoint.x(),taskPoint.y(),taskPoint.z());
    if(taskList.size() == numOfRobots)
    {
        RCLCPP_INFO(this->get_logger(),"start allocate");
        ///get cost martrix
        ///step1: get robot current position
        std::vector<Vector3d> CurrPosList;
        for(int i = 0; i < numOfRobots; i++)
        {
            geometry_msgs::msg::TransformStamped t;
            std::string fromFrameRel = names.at(i)+"_base_link";
            std::string toFrameRel = "map";
            Vector3d CurrPos(0,0,0);
            try 
            {
                t = tf_buffer->lookupTransform(toFrameRel, fromFrameRel,tf2::TimePointZero);
                CurrPos(0) = t.transform.translation.x;
                CurrPos(1) = t.transform.translation.y;
                CurrPos(2) = t.transform.translation.z;
            } 
            catch (const tf2::TransformException & ex) 
            {
                RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
                return;
            }
            CurrPosList.push_back(CurrPos);
        }
        ///step2: combinate all pairs
        MatrixXd costMat(numOfRobots,taskList.size());
        for(int i = 0; i < numOfRobots; i++)
        {
            for(int j = 0; j < taskList.size(); j++)
            {
                pfSolver.resetUsedGrids();
                costMat(i,j) = pfSolver.graphSearch(CurrPosList.at(i),taskList.at(j));
            }
        }
        ///TODO: allocate here return taskAlloc
        double res = allocator.Solve(costMat,taskAlloc);
        ///TODO: publish res
        if(res > 0)
        {
            for(int i = 0; i < numOfRobots; i++)
            {
                geometry_msgs::msg::PoseStamped cmd;
                Vector3d task = taskList.at(taskAlloc.at(i));
                cmd.pose.position.x = task(0);
                cmd.pose.position.y = task(1);
                cmd.pose.position.z = task(2);
                pubGoals.at(i)->publish(cmd);
            }
        }

        taskAlloc.clear();
        taskList.clear();
    }
}
