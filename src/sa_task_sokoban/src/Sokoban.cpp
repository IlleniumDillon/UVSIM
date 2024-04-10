#include "Sokoban.hpp"

class graphSearchPrivate
{
public:
    void updateMap(nav_msgs::msg::OccupancyGrid map_dilate)
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
    //nav_msgs::msg::OccupancyGrid map_dilate;
};
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
class hungarianPrivate
{
public:
};
class taskExecutorPrivate
{
public:
    taskExecutorPrivate(double width, double hight, double wallthickness, double ori_x, double ori_y, double res = 0.05)
    {
        resolution = res;

        int edgeThickness = wallthickness / res;
        int mapWidth = width / res - edgeThickness;
        int mapHight = hight / res - edgeThickness;

        cv::Mat temp = cv::Mat(mapWidth,mapHight,CV_8UC1,cv::Scalar(0));
        cv::copyMakeBorder(temp,MapBlank,edgeThickness,edgeThickness,edgeThickness,edgeThickness,cv::BORDER_CONSTANT,255);

        origin_x = - hight / 2 + ori_x;
        origin_y = - width / 2 + ori_y;
        origin_z = 0;
    }
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
    void generateGoal(std::shared_ptr<graphSearchPrivate> pTool)
    {
        goalPosList.clear();
        goalRPYList.clear();

        std::string modelName = taskList.model_names.at(curTaskIndx);
        int indx = -1;
        for(int i = 0; i < model_state.model_names.size(); i++)
        {
            if(modelName == model_state.model_names.at(i))
            {
                indx = i;
                break;
            }
        }
        if(indx == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("generateGoal"),"no model named:%s",modelName.c_str());
            return;
        }
        geometry_msgs::msg::Pose curState = model_state.model_poses.at(indx);
        geometry_msgs::msg::Pose tarState = taskList.model_poses.at(curTaskIndx);

        Vector3d boxCurPos(curState.position.x,curState.position.y,curState.position.z);
        Vector3d boxTarPos(tarState.position.x,tarState.position.y,tarState.position.z);

        double res = pTool->solver.graphSearch(boxCurPos,boxTarPos);
        goalPosList = pTool->solver.getPath();
        pTool->solver.resetUsedGrids();

        if(res < 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("lll"),"lll");
            return;
        }
        
        for(int i = 0; i < goalPosList.size()-1; i++)
        {
            RCLCPP_INFO(rclcpp::get_logger("goalList"),"%f,%f",goalPosList.at(i).x(),goalPosList.at(i).y());
            int j = i+1;
            double x1 = goalPosList.at(i).x();
            double y1 = goalPosList.at(i).y();
            double x2 = goalPosList.at(j).x();
            double y2 = goalPosList.at(j).y();
            double yaw = atan2(y2-y1,x2-x1);
            goalRPYList.push_back(Vector3d(0,0,yaw));
            goalPosList.at(i).x() -= length_m*cos(yaw);
            goalPosList.at(i).y() -= length_m*sin(yaw);
        }
    }
    nav_msgs::msg::OccupancyGrid generateMap(bool ignoreFlag = false)
    {
        auto ignoreNames = ignore.model_names;

        cv::Mat MapFull = MapBlank.clone();
        int objectNum = model_state.model_names.size();
        std::vector<std::vector<cv::Point>>objVertexs;

        for(int i = 0; i < objectNum; i++)
        {
            if(ignoreFlag && ignoreNames.end()!=std::find(ignoreNames.begin(),ignoreNames.end(),model_state.model_names.at(i))) continue;
            std::string substr = model_state.model_names.at(i).substr(0,3);
            if(substr == "obs")
            {
                std::vector<cv::Point> objVertex;
                geometry_msgs::msg::Quaternion q = model_state.model_poses.at(i).orientation;
                tf2::Quaternion quat;
                tf2::convert(q, quat);
                tf2::Matrix3x3 m(quat);
                double r,p,y;
                m.getRPY(r,p,y);
                double ang[4] = {CV_PI/4,CV_PI/4*3,-CV_PI/4*3,-CV_PI/4};
                double rl = sqrt(2)*0.25;
                for(int j = 0; j < 4; j++)
                {
                    cv::Point temp;
                    temp.x = (rl*cos(ang[j]+y) + model_state.model_poses.at(i).position.x - origin_x) / resolution;
                    temp.y = (rl*sin(ang[j]+y) + model_state.model_poses.at(i).position.y - origin_y) / resolution;
                    objVertex.push_back(temp);
                }
                objVertexs.push_back(objVertex);
            }
        }

        cv::fillPoly(MapFull,objVertexs,cv::Scalar(255));
        cv::Mat MapDilate;
        cv::dilate(MapFull,MapDilate,cv::getStructuringElement(cv::MorphShapes::MORPH_RECT,cv::Size(10,10)));

        nav_msgs::msg::OccupancyGrid map_dil;
        map_dil.header.frame_id="map_dil";
        map_dil.info.height = MapBlank.rows;
        map_dil.info.width = MapBlank.cols;
        map_dil.info.resolution = resolution;
        map_dil.info.origin.position.x = origin_x;
        map_dil.info.origin.position.y = origin_y;
        map_dil.info.origin.position.z = origin_z;
        
        for(int row = 0; row < MapBlank.rows; row++)
        {
            for(int col = 0; col < MapBlank.cols; col++)
            {
                map_dil.data.push_back(MapDilate.at<uchar>(row,col) / 255 * 100);
            }
        }

        return map_dil;
    }

    simbridge::msg::ModelState model_state;
    simbridge::msg::ModelState taskList;

    cv::Mat MapBlank;
    double resolution = 0.05;
    double origin_x,origin_y,origin_z;

    std::vector<Vector3d> goalPosList;
    std::vector<Vector3d> goalRPYList;
    int curTaskIndx = -1;

    simbridge::msg::ModelIgnore ignore;
    std_msgs::msg::Float64 arm_arm;
    std_msgs::msg::Float64 arm_hand;

    double length_m = 0.7;
};

Sokoban::Sokoban() 
    :Node("sokoban"),
    impl_graphsearcher(std::make_shared<graphSearchPrivate>()),
    impl_apfsolver(std::make_shared<APFSolverPrivate>()),
    impl_hungarian(std::make_shared<hungarianPrivate>()),
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
                impl_taskexecutor->taskList.model_names.clear();
                impl_taskexecutor->taskList.model_poses.clear();
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
        static bool queryMapInProgress = false;
        if(lastState == Idle)
        {
            RCLCPP_INFO(this->get_logger(),"Current state: Shift; update map");
            impl_graphsearcher->updateMap(impl_taskexecutor->generateMap(true));
            /// get goal
            impl_taskexecutor->generateGoal(impl_graphsearcher);
            Vector3d goalPos = impl_taskexecutor->goalPosList.front();
            Vector3d goalRPY = impl_taskexecutor->goalRPYList.front();

            /// get path
            impl_graphsearcher->updateMap(impl_taskexecutor->generateMap(false));
            impl_graphsearcher->solver.graphSearch(CurrPos,goalPos);
            /// set path to apf
            impl_apfsolver->setGoalRPY(goalRPY);
            impl_apfsolver->setPath(impl_graphsearcher->solver.getPath());
            impl_graphsearcher->solver.resetUsedGrids();
        }
        else
        {
            /// apf update cmd_vel
            impl_apfsolver->update(CurrPos,CurrRPY);
            /// when exit, switch to Activate
            if(impl_apfsolver->reachFlag)
            {
                RCLCPP_INFO(this->get_logger(),"Current state: Shift; done");
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
            if(activating > 25)
            {
                simbridge::msg::AddJoint addmsg;
                addmsg.model1 = impl_taskexecutor->taskList.model_names.at(impl_taskexecutor->curTaskIndx);
                addmsg.link1 = "link_0";
                addmsg.model2 = robotName;
                addmsg.link2 = robotName+"_"+armName+"_hand_link";
                addmsg.joint_state = true;
                pub_addjoint->publish(addmsg);
                curState = ShiftWithBox;
                RCLCPP_INFO(this->get_logger(),"Current state: Activate; Done");
            }
        }
        lastState = Activate;
        break;
    }
    case ShiftWithBox:
    {
        if(lastState == Activate)
        {
            impl_apfsolver->setGoalRPY(impl_taskexecutor->goalRPYList.back());
            impl_apfsolver->setPath(impl_taskexecutor->goalPosList);
        }
        else
        {
            /// apf update cmd_vel
            impl_apfsolver->update(CurrPos,CurrRPY);
            /// when exit, switch to Activate
            if(impl_apfsolver->reachFlag)
            {
                RCLCPP_INFO(this->get_logger(),"Current state: ShiftWithBox; done");
                curState = Deactivate;
            }
        }
        lastState = ShiftWithBox;
        break;
    }
    case Deactivate:
    {
        static int deactivating = 0;
        if(lastState != Deactivate)
        {
            deactivating = 0;
            simbridge::msg::AddJoint addmsg;
            addmsg.model1 = impl_taskexecutor->taskList.model_names.at(impl_taskexecutor->curTaskIndx);
            addmsg.link1 = "link_0";
            addmsg.model2 = robotName;
            addmsg.link2 = robotName+"_"+armName+"_hand_link";
            addmsg.joint_state = false;
            pub_addjoint->publish(addmsg);
            impl_taskexecutor->deactivateArm();
            RCLCPP_INFO(this->get_logger(),"Current state: Deactivate; deactivating");
        }
        else
        {
            deactivating ++;
            if(deactivating > 25)
            {
                curState = Idle;
                RCLCPP_INFO(this->get_logger(),"Current state: Deactivate; Done");
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
    //pub_model_ignore->publish(impl_taskexecutor->ignore);
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
