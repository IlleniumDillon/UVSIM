#include "WorldMapUpdater.hpp"
#include <algorithm> 

WorldMapUpdater::WorldMapUpdater(double width, double hight, double wallthickness, double ori_x, double ori_y, double res) : Node("WorldMapUpdater")
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

    RCLCPP_INFO(this->get_logger(),"Map Size:[%d,%d]",MapBlank.rows,MapBlank.cols);

    sub_model = this->create_subscription<simbridge::msg::ModelState>(
        "model_states",1,std::bind(&WorldMapUpdater::modelStateCallback,this,std::placeholders::_1)
    );
    pub_ori = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map",1);

    srv_map = this->create_service<simbridge::srv::QueryMap>(
        "query_map", std::bind(&WorldMapUpdater::queryMapServace,this,std::placeholders::_1,std::placeholders::_2)
    );
}

void WorldMapUpdater::modelStateCallback(const simbridge::msg::ModelState::SharedPtr msg)
{
    if(msg->model_names.size()!=msg->model_poses.size())
    {
        RCLCPP_ERROR(this->get_logger(),"model name list mismatch model poses list");
        return;
    }

    modelStates = *msg;

    cv::Mat MapFull = MapBlank.clone();
    int objectNum = msg->model_names.size();
    std::vector<std::vector<cv::Point>>objVertexs;
    for(int i = 0; i < objectNum; i++)
    {
        if(ignoreNames.end()!=std::find(ignoreNames.begin(),ignoreNames.end(),msg->model_names.at(i))) continue;
        std::string substr = msg->model_names.at(i).substr(0,3);
        if(substr == "obs")
        {
            std::vector<cv::Point> objVertex;
            geometry_msgs::msg::Quaternion q = msg->model_poses.at(i).orientation;
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
                temp.x = (rl*cos(ang[j]+y) + msg->model_poses.at(i).position.x - origin_x) / resolution;
                temp.y = (rl*sin(ang[j]+y) + msg->model_poses.at(i).position.y - origin_y) / resolution;
                objVertex.push_back(temp);
            }
            objVertexs.push_back(objVertex);
        }
    }
    cv::fillPoly(MapFull,objVertexs,cv::Scalar(255));

    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id="map";
    map.info.height = MapBlank.rows;
    map.info.width = MapBlank.cols;
    map.info.resolution = resolution;
    map.info.origin.position.x = origin_x;
    map.info.origin.position.y = origin_y;
    map.info.origin.position.z = origin_z;
    
    for(int row = 0; row < MapBlank.rows; row++)
    {
        for(int col = 0; col < MapBlank.cols; col++)
        {
            map.data.push_back(MapFull.at<uchar>(row,col) / 255 * 100);
        }
    }

    pub_ori->publish(map);
}

void WorldMapUpdater::queryMapServace(const simbridge::srv::QueryMap_Request::SharedPtr request, simbridge::srv::QueryMap_Response::SharedPtr responce)
{
    ignoreNames = request->ignore_model_names;

    cv::Mat MapFull = MapBlank.clone();
    int objectNum = modelStates.model_names.size();
    std::vector<std::vector<cv::Point>>objVertexs;

    for(int i = 0; i < objectNum; i++)
    {
        if(ignoreNames.end()!=std::find(ignoreNames.begin(),ignoreNames.end(),modelStates.model_names.at(i))) continue;
        std::string substr = modelStates.model_names.at(i).substr(0,3);
        if(substr == "obs")
        {
            std::vector<cv::Point> objVertex;
            geometry_msgs::msg::Quaternion q = modelStates.model_poses.at(i).orientation;
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
                temp.x = (rl*cos(ang[j]+y) + modelStates.model_poses.at(i).position.x - origin_x) / resolution;
                temp.y = (rl*sin(ang[j]+y) + modelStates.model_poses.at(i).position.y - origin_y) / resolution;
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

    responce->map = map_dil;
}
