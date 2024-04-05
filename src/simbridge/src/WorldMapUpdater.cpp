#include "WorldMapUpdater.hpp"

class SimObject
{

};

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

    sub_ = this->create_subscription<simbridge::msg::ModelState>(
        "model_states",1,std::bind(&WorldMapUpdater::modelStateCallback,this,std::placeholders::_1)
    );
    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map",1);
}

void WorldMapUpdater::modelStateCallback(const simbridge::msg::ModelState::SharedPtr msg)
{
    if(msg->model_names.size()!=msg->model_poses.size())
    {
        RCLCPP_ERROR(this->get_logger(),"model name list mismatch model poses list");
        return;
    }
    cv::Mat MapFull = MapBlank.clone();
    int objectNum = msg->model_names.size();
    std::vector<std::vector<cv::Point>>objVertexs;
    for(int i = 0; i < objectNum; i++)
    {
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

    pub_->publish(map);
}
