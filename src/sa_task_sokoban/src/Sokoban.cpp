#include "Sokoban.hpp"

class graphSearchPrivate
{
public:
    void updateMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_dilate = *msg;
    }
    void updateMap()
    {
        solver.deleteGridMap();
        Vector3d ol(map_dilate.info.origin.position.x,
            map_dilate.info.origin.position.y,
            map_dilate.info.origin.position.z);
        Vector3d ou(map_dilate.info.origin.position.x,
            map_dilate.info.origin.position.y,
            map_dilate.info.origin.position.z+map_dilate.info.resolution);
        solver.initGridMap(map_dilate.info.resolution,ol,ou,map_dilate.info.width,map_dilate.info.height,1);
        RCLCPP_INFO(rclcpp::get_logger("grapheSearcher"),"%d,%d",map_dilate.info.height,map_dilate.info.width);
        for(int i = 0; i < map_dilate.info.height; i++)
        {
            for(int j = 0; j < map_dilate.info.width; j++)
            {
                //RCLCPP_INFO(this.get_logger(),"%d",map_dilate.data.at(i*map_dilate.info.width+j));
                if(map_dilate.data.at(i*map_dilate.info.width+j) == -1||
                    map_dilate.data.at(i*map_dilate.info.width+j) == 100)
                {
                    solver.setObs(j,i,0);
                }
                
            }
        }
    }
    gridPathFinder solver;
    nav_msgs::msg::OccupancyGrid map_dilate;
};
class APFSolverPrivate
{
public:
    void setPath(std::vector<Vector3d> p)
    {
        path = p;
    }
    void setGoalRPY(Vector3d rpy)
    {
        goalRPY = rpy;
    }
    void update()
    {

    }
    void updateScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan = *msg;
    }
    sensor_msgs::msg::LaserScan scan;
    std::vector<Vector3d> path;
    Vector3d goalRPY;
    geometry_msgs::msg::Twist cmd_vel;
    bool reachFlag = false;
};
class hungarianPrivate
{
public:
};
class taskExecutorPrivate
{
public:
    void updateModelCallback(const simbridge::msg::ModelState::SharedPtr msg)
    {
        model_state = *msg;
    }
    void updateTaskCallback(const simbridge::msg::ModelState::SharedPtr msg)
    {
        taskList = *msg;
    }
    void activateArm()
    {
        arm_arm.data = 0;
        arm_hand.data = 0;
    }
    void deactivateArm()
    {
        arm_arm.data = -2;
        arm_hand.data = 2;
    }
    void generateGoal()
    {

    }
    simbridge::msg::ModelState model_state;
    simbridge::msg::ModelState taskList;
    Vector3d goalPos;
    Vector3d goalRPY;
    int curTaskIndx = -1;

    simbridge::msg::ModelIgnore ignore;
    std_msgs::msg::Float64 arm_arm;
    std_msgs::msg::Float64 arm_hand;
};

Sokoban::Sokoban() 
    :Node("sokoban"),
    impl_graphsearcher(std::make_shared<graphSearchPrivate>()),
    impl_apfsolver(std::make_shared<APFSolverPrivate>()),
    impl_hungarian(std::make_shared<hungarianPrivate>()),
    impl_taskexecutor(std::make_shared<taskExecutorPrivate>())
{
    this->declare_parameter("robot_name", "robot0");
    this->declare_parameter("arm_name", "arm");
    robotName = this->get_parameter("robot_name").as_string();
    armName = this->get_parameter("arm_name").as_string();
    fromFrameRel = robotName+"_base_link";
    toFrameRel = "map";

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
    pub_arm_arm_joint = this->create_publisher<std_msgs::msg::Float64>(armName+"_arm",1);
    pub_arm_hand_joint = this->create_publisher<std_msgs::msg::Float64>(armName+"_hand",1);
    pub_model_ignore = this->create_publisher<simbridge::msg::ModelIgnore>("/model_ignore",1);

    sub_model_state = this->create_subscription<simbridge::msg::ModelState>(
        "/model_state",1,std::bind(&taskExecutorPrivate::updateModelCallback,impl_taskexecutor,std::placeholders::_1)
    );
    sub_task = this->create_subscription<simbridge::msg::ModelState>(
        "/task",1,std::bind(&taskExecutorPrivate::updateTaskCallback,impl_taskexecutor,std::placeholders::_1)
    );
    sub_map_dilate = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map_dil",1,std::bind(&graphSearchPrivate::updateMapCallback,impl_graphsearcher,std::placeholders::_1)
    );
    sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",1,std::bind(&APFSolverPrivate::updateScanCallback,impl_apfsolver,std::placeholders::_1)
    );

    timer = this->create_wall_timer(100ms,std::bind(&Sokoban::timerCallback,this));
}

void Sokoban::timerCallback()
{
    updatePose();
    switch (curState)
    {
    case BeforeInit:
    {
        RCLCPP_INFO(this->get_logger(),"Current state: BeforeInit");

        impl_taskexecutor->ignore.model_names.clear();
        impl_taskexecutor->deactivateArm();
        impl_apfsolver->cmd_vel.angular.x = 0;
        impl_apfsolver->cmd_vel.angular.y = 0;
        impl_apfsolver->cmd_vel.angular.z = 0;
        impl_apfsolver->cmd_vel.linear.x = 0;
        impl_apfsolver->cmd_vel.linear.y = 0;
        impl_apfsolver->cmd_vel.linear.z = 0;

        curState = Idle;
        lastState = BeforeInit;
        break;
    }
    case Idle:
    {
        if(lastState == BeforeInit)
        {
            RCLCPP_INFO(this->get_logger(),"Current state: Idle; waiting task list");
        }
        else if(lastState == Deactivate)
        {
            RCLCPP_INFO(this->get_logger(),"Current state: Idle; task complete");
            impl_taskexecutor->curTaskIndx++;
            if(impl_taskexecutor->curTaskIndx >= impl_taskexecutor->taskList.model_names.size())
            {
                RCLCPP_INFO(this->get_logger(),"Current state: Idle; all task complete");
                impl_taskexecutor->curTaskIndx = -1;
                curState = BeforeInit;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),"Current state: Idle; turn to next task");
                impl_taskexecutor->ignore.model_names.clear();
                impl_taskexecutor->ignore.model_names.push_back(
                    impl_taskexecutor->taskList.model_names.at(
                        impl_taskexecutor->curTaskIndx
                    )
                );
                curState = Shift;
            }
        }
        else
        {
            if(impl_taskexecutor->taskList.model_names.size() > 0)
            {
                RCLCPP_INFO(this->get_logger(),"Current state: Idle; start");
                impl_taskexecutor->curTaskIndx = 0;
                impl_taskexecutor->ignore.model_names.clear();
                impl_taskexecutor->ignore.model_names.push_back(
                    impl_taskexecutor->taskList.model_names.at(
                        impl_taskexecutor->curTaskIndx
                    )
                );
                curState = Shift;
            }
        }
        lastState = Idle;
        break;
    }
    case Shift:
    {
        static bool updateMap = false;
        if(lastState == Idle)
        {
            updateMap = true;
        }
        else
        {
            if(updateMap == true)
            {
                updateMap = false;
                impl_graphsearcher->updateMap();
                /// get goal
                impl_taskexecutor->generateGoal();
                Vector3d goalPos = impl_taskexecutor->goalPos;
                Vector3d goalRPY = impl_taskexecutor->goalRPY;
                /// get path
                impl_graphsearcher->solver.resetUsedGrids();
                impl_graphsearcher->solver.graphSearch(CurrPos,goalPos);
                /// set path to apf
                impl_apfsolver->setGoalRPY(goalRPY);
                impl_apfsolver->setPath(impl_graphsearcher->solver.getPath());
            }
            /// apf update cmd_vel
            impl_apfsolver->update();
            /// when exit, switch to Activate
            if(impl_apfsolver->reachFlag)
            {
                curState = Activate;
            }
        }
        lastState = Shift;
        break;
    }
    case Activate:
    {
        static int activating = 0;
        if(lastState != Activate)
        {
            activating = 0;
            impl_taskexecutor->activateArm();
            RCLCPP_INFO(this->get_logger(),"Current state: Activate; activating");
        }
        else
        {
            activating ++;
            if(activating > 15)
            {
                curState = ShiftWithBox;
                RCLCPP_INFO(this->get_logger(),"Current state: Activate; Done");
            }
        }
        lastState = Activate;
        break;
    }
    case ShiftWithBox:
    {
        lastState = ShiftWithBox;
        break;
    }
    case Deactivate:
    {
        static int deactivating = 0;
        if(lastState != Activate)
        {
            deactivating = 0;
            impl_taskexecutor->deactivateArm();
            RCLCPP_INFO(this->get_logger(),"Current state: Activate; activating");
        }
        else
        {
            deactivating ++;
            if(deactivating > 15)
            {
                curState = Idle;
                RCLCPP_INFO(this->get_logger(),"Current state: Activate; Done");
            }
        }
        lastState = Deactivate;
        break;
    }
    default:
    {
        RCLCPP_ERROR(this->get_logger(),"undefined state");
        curState = BeforeInit;
        break;
    }
    }
    pub_model_ignore->publish(impl_taskexecutor->ignore);
    pub_arm_arm_joint->publish(impl_taskexecutor->arm_arm);
    pub_arm_hand_joint->publish(impl_taskexecutor->arm_hand);
    pub_cmd_vel->publish(impl_apfsolver->cmd_vel);
}

void Sokoban::updatePose()
{
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
    } 
    catch (const tf2::TransformException & ex) 
    {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
    }
}
