#include "Sokoban.hpp"

class APFSolverPrivate
{
public:
    void setPath(std::vector<Vector3d> p)
    {
        path = p;
        updateFlag = 0;
    }
    void setGoalRPY(Vector3d rpy)
    {
        goalRPY = rpy;
    }
    void update(Vector3d& curPos, Vector3d& curRPY)
    {
        Vector3d err3d = curPos - path.back();
        Vector2d err(err3d.x(),err3d.y());
        double yawerr = curRPY.z() - goalRPY.z();

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
        int numOfPoint = forward - minIndx;
        double fx,fy;

        if(updateFlag == 0)
        {
            reachFlag = false;
            if(err.norm() < 0.1) updateFlag++;
            if(numOfPoint == 10)
            {
                for(int indx = minIndx; indx < forward; indx++)
                {
                    Ftal += gravitation(path.at(indx),curPos,3);
                }
                Matrix3d rotation_matrix(AngleAxisd(-curRPY(2),Vector3d::UnitZ()));
                Ftal=rotation_matrix*Ftal;
                
                fx = Ftal(0),fy = Ftal(1);
            }
            else
            {
                Ftal += gravitation(path.back(),curPos,3);
                Matrix3d rotation_matrix(AngleAxisd(-curRPY(2),Vector3d::UnitZ()));
                Ftal=rotation_matrix*Ftal;
                
                fx = Ftal(0),fy = Ftal(1);
                if(abs(fy) > 0.05)
                {
                    fx = 0;
                }
            }
        }
        else if(updateFlag == 1)
        {
            reachFlag = false;
            if(abs(yawerr) < 0.05) updateFlag++;
            fy = -yawerr * 10;
            if(fy > 0.3) fy = 0.3;
            if(fy < -0.3) fy = -0.3;
            fx = 0;
        }
        else
        {
            fx = 0;
            fy = 0;
            updateFlag = 0;
            reachFlag = true;
        }

        if(fx > 0.3) fx = 0.3;
        if(fx < -0.3) fx = -0.3;
        if(fy > 0.3) fy = 0.3;
        if(fy < -0.3) fy = -0.3;

        cmd_vel.linear.x = fx;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = fy;
    }
    void updateScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        laserData = *msg;
    }
    sensor_msgs::msg::LaserScan laserData;
    std::vector<Vector3d> path;
    Vector3d goalRPY;
    geometry_msgs::msg::Twist cmd_vel;
    bool reachFlag = false;
private:
    Vector3d gravitation(Vector3d& source, Vector3d& object, double katt)
    {
        Vector3d P0 = object - source;
        Vector3d Fatt = -katt * P0;
        return Fatt;
    }
    Vector3d repulsion(Vector3d &source, Vector3d &object, float& dis, double krep, double range)
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
    int updateFlag = 0;
};

class taskExecutorPrivate
{
public:
    taskExecutorPrivate(double width, double hight, double wallthickness, double ori_x, double ori_y, double res = 0.05)
    {
        resolution = res;

        int edgeThickness = round((double)wallthickness / res);
        int mapWidth = round((double)width / res - edgeThickness * 2);
        int mapHight = round((double)hight / res - edgeThickness * 2);

        cv::Mat temp = cv::Mat(mapWidth,mapHight,CV_8UC1,cv::Scalar(0));
        cv::copyMakeBorder(temp,MapBlank,edgeThickness,edgeThickness,edgeThickness,edgeThickness,cv::BORDER_CONSTANT,255);

        // origin_x = - hight / 2 + ori_x;
        // origin_y = - width / 2 + ori_y;
        origin_x = -0.6*10 - 0.3;
        origin_y = -0.6*7 - 0.3;
        origin_z = 0;

        cout << "origin: " << origin_x << " " << origin_y << " " << origin_z << endl;

        int sokomapWidth = round(mapHight * res / 0.6) + 2;
        int sokomapHight = round(mapWidth * res / 0.6) + 2;
        RCLCPP_INFO(rclcpp::get_logger("task_exe"),"%d,%d",sokomapHight,sokomapWidth);
        RCLCPP_INFO(rclcpp::get_logger("task_exe"),"%d,%d,%d",mapWidth,mapHight,edgeThickness);
        world = World(sokomapWidth,sokomapHight);
        //world.setShowSize(600,800);
        for (int i = 0; i < sokomapHight; i++) 
        {
            for (int j = 0; j < sokomapWidth; j++) 
            {
                if(i == 0 || i == sokomapHight-1 || j == 0 || j == sokomapWidth-1)
                {
                    world.addBox(Box(Vector2i(j, i), "wall" + std::to_string(world.boxes.size()), true));
                }
            }
        }
    }
    void updateModelCallback(const simbridge::msg::ModelState::SharedPtr msg)
    {
        static bool first = true;
        model_state = *msg;
        if (first)
        {
            for (int i = 0; i < model_state.model_names.size(); i++)
            {
                if (model_state.model_names.at(i).substr(0, 3) == "rob")
                {
                    Vector2i robotPos = Vector2i(
                        round((model_state.model_poses.at(i).position.x - origin_x) / 0.6),
                        world.height - round((model_state.model_poses.at(i).position.y - origin_y) / 0.6) - 1
                    );
                    cout << robotPos.x() << " " << robotPos.y() << endl;
                    world.addRobot(Robot(robotPos, model_state.model_names.at(i)));
                }
                else if (model_state.model_names.at(i).substr(0, 3) == "obs")
                {
                    Vector2i boxPos = Vector2i(
                        round((model_state.model_poses.at(i).position.x - origin_x) / 0.6),
                        world.height - round((model_state.model_poses.at(i).position.y - origin_y) / 0.6) - 1
                    );
                    world.addBox(Box(boxPos, model_state.model_names.at(i), true));
                }
            }

            first = false;
        }
    }
    void updateTaskCallback(const simbridge::msg::ModelState::SharedPtr msg)
    {
        taskList = *msg;
        for (int i = 0; i < taskList.model_names.size(); i++)
        {
            Vector2i taskPos = Vector2i(
                round((model_state.model_poses.at(i).position.x - origin_x) / 0.6),
                world.height - round((model_state.model_poses.at(i).position.y - origin_y) / 0.6) - 1
            );
            cout << taskPos.x() << " " << taskPos.y() << endl;
            world.addTask(Task(taskPos, taskList.model_names.at(i),""));

            //find box
            for(auto& box : world.boxes)
            {
                if(box.name == taskList.model_names.at(i))
                {
                    box.static_ = false;
                }
            }
        }
        sokobansolver.setWorld(&world);
        auto robotActionList = sokobansolver.pathList;

        Robot::Action lastAction = Robot::Action::NOACTION;
        vector<Vector2d> onePath;
        Vector2d robotLastPos = 0.6 * Vector2d(
            world.robots.at(0).position.x() + 0.5,
            world.robots.at(0).position.y() + 0.5);
        for (int i = 0; i < robotActionList.size(); i++)
        {
            auto robotAction = robotActionList.at(i).action;
            auto robotMove = robotActionList.at(i).move;
            auto robot = world.robots.at(0);
            if (robotAction != lastAction)
            {
                pathList.push_back(onePath);
                onePath.clear();
                //get rpy
                Vector3d goalRPY;
                if (robotMove.x() == 1)
                {
                    goalRPY = Vector3d(0,0,0);
                }
                else if (robotMove.x() == -1)
                {
                    goalRPY = Vector3d(0,0,M_PI);
                }
                else if (robotMove.y() == 1)
                {
                    goalRPY = Vector3d(0,0,M_PI/2);
                }
                else if (robotMove.y() == -1)
                {
                    goalRPY = Vector3d(0,0,-M_PI/2);
                }
                goalRPYList.push_back(goalRPY);

                lastAction = robotAction;
                cout << "action: " << robotAction << endl;
                i--;
                continue;
            }
            robotLastPos += 0.6 * Vector2d(robotMove.x(), robotMove.y());
            onePath.push_back(robotLastPos);
            cout << robotLastPos.x() + origin_x << " " << robotLastPos.y() + origin_y << endl;
        }
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

    simbridge::msg::ModelState model_state;
    simbridge::msg::ModelState taskList;

    cv::Mat MapBlank;
    double resolution = 0.05;
    double origin_x,origin_y,origin_z;

    simbridge::msg::ModelIgnore ignore;
    std_msgs::msg::Float64 arm_arm;
    std_msgs::msg::Float64 arm_hand;

    double length_m = 0.7;

    World world;
    SokobanSolver sokobansolver;

    vector<vector<Vector2d>> pathList;
    vector<bool> actionList;
    vector<vector<Vector3d>> goalPosList;
    vector<Vector3d> goalRPYList;
    int curTaskIndx = -1;
    int curPathIndx = -1;
    int curPointIndx = -1;
};

Sokoban::Sokoban() 
    :Node("sokoban"),
    impl_apfsolver(std::make_shared<APFSolverPrivate>()),
    impl_taskexecutor(std::make_shared<taskExecutorPrivate>(8.7,11.7,0.15,-0.3,0,0.05))
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
    //pub_model_ignore = this->create_publisher<simbridge::msg::ModelIgnore>("/model_ignore",1);
    pub_addjoint = this->create_publisher<simbridge::msg::AddJoint>("/add_joint",1);

    sub_model_state = this->create_subscription<simbridge::msg::ModelState>(
        "/model_states",1,std::bind(&taskExecutorPrivate::updateModelCallback,impl_taskexecutor,std::placeholders::_1)
    );
    sub_task = this->create_subscription<simbridge::msg::ModelState>(
        "/task",1,std::bind(&taskExecutorPrivate::updateTaskCallback,impl_taskexecutor,std::placeholders::_1)
    );
    // sub_map_dilate = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    //     "/map_dil",1,std::bind(&graphSearchPrivate::updateMapCallback,impl_graphsearcher,std::placeholders::_1)
    // );
    sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",1,std::bind(&APFSolverPrivate::updateScanCallback,impl_apfsolver,std::placeholders::_1)
    );

    timer = this->create_wall_timer(1s,std::bind(&Sokoban::timerCallback,this));
}

void Sokoban::timerCallback()
{
    /*updatePose();
    
    pub_arm_arm_joint->publish(impl_taskexecutor->arm_arm);
    pub_arm_hand_joint->publish(impl_taskexecutor->arm_hand);
    pub_cmd_vel->publish(impl_apfsolver->cmd_vel);*/
    if (!impl_taskexecutor->sokobansolver.pathList.empty())
    {
        impl_taskexecutor->world.draw();
        impl_taskexecutor->sokobansolver.update();
        impl_taskexecutor->world.update();
        
        cv::waitKey(1);
    }
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
