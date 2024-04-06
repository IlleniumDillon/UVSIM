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

    }
    gridPathFinder solver;
    nav_msgs::msg::OccupancyGrid map_dilate;
};
class APFSolverPrivate
{
public:
    geometry_msgs::msg::Twist cmd_vel;
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
    void DeactivateArm()
    {
        arm_arm.data = -2;
        arm_hand.data = 2;
    }
    simbridge::msg::ModelState model_state;
    simbridge::msg::ModelState taskList;
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
    pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
    pub_arm_arm_joint = this->create_publisher<std_msgs::msg::Float64>("arm_arm",1);
    pub_arm_hand_joint = this->create_publisher<std_msgs::msg::Float64>("arm_hand",1);
    pub_model_ignore = this->create_publisher<simbridge::msg::ModelIgnore>("model_ignore",1);

    sub_model_state = this->create_subscription<simbridge::msg::ModelState>(
        "model_state",1,std::bind(&taskExecutorPrivate::updateModelCallback,impl_taskexecutor,std::placeholders::_1)
    );
    sub_task = this->create_subscription<simbridge::msg::ModelState>(
        "task",1,std::bind(&taskExecutorPrivate::updateTaskCallback,impl_taskexecutor,std::placeholders::_1)
    );
    sub_map_dilate = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map_dil",1,std::bind(&graphSearchPrivate::updateMapCallback,impl_graphsearcher,std::placeholders::_1)
    );

    timer = this->create_wall_timer(100ms,std::bind(&Sokoban::timerCallback,this));
}

void Sokoban::timerCallback()
{
    switch (curState)
    {
    case BeforeInit:
    {
        RCLCPP_INFO(this->get_logger(),"Current state: BeforeInit");

        impl_taskexecutor->ignore.model_names.clear();
        impl_taskexecutor->DeactivateArm();
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
                ///TODO: get goal
                ///TODO: get path
                ///TODO: set path to apf
            }
            ///TODO: apf update cmd_vel
            ///TODO: when exit, switch to Activate
        }
        lastState = Shift;
        break;
    }
    case Activate:
    {
        static int activating = 0;
        if(lastState == Shift)
        {
            activating = 0;
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

