#include "rrt.h"
namespace UV
{
int signum(double x)
{
    return x == 0 ? 0 : x < 0 ? -1
                              : 1;
}

double mod(double value, double modulus)
{
    return fmod(fmod(value, modulus) + modulus, modulus);
}

double intbound(double s, double ds)
{
    // Find the smallest positive t such that s+t*ds is an integer.
    if (ds < 0)
    {
        return intbound(-s, -ds);
    }
    else
    {
        s = mod(s, 1);
        // problem is now s+t*ds = 1
        return (1 - s) / ds;
    }
}

double RRT::res_distance(double p,double direction)
{
    if(direction>0)
    {
        return (1-fmod(p,1));
    }
    else
    {
        return -fmod(p,1);
    }
}

void RRT::SetMap(double _resolution, int max_x_id, int max_y_id, int max_z_id,Vector3d _origin)
{
    resolution = _resolution;
    inv_resolution = 1/resolution;
    mapX = max_x_id;
    mapY = max_y_id;
    mapZ = max_z_id;
    origin = _origin;

    goal_gen = std::mt19937(std::random_device{}());
    goal_dis = std::uniform_int_distribution<int>(0, 100);
    x_dis = std::uniform_real_distribution<double>(0, mapX*resolution);
    y_dis = std::uniform_real_distribution<double>(0, mapY*resolution);
    z_dis = std::uniform_real_distribution<double>(0, mapZ*resolution);  

    //RCLCPP_INFO(rclcpp::get_logger("rrt"),"resolution:%f,mapXYZ:%d,%d,%d,origin:%f,%f,%f",
                // resolution,mapX,mapY,mapZ,origin[0],origin[1],origin[2]);

    
}

void RRT::ResetRRT(Vector3d start, Vector3d goal)
{
    node_list.clear();
    start_data = start;
    goal_data = goal;
    RRT_Node* start_node = new RRT_Node(start_data);
    node_list.push_back(start_node);
    
}

RRT_Node* RRT::Sample()
{
    RRT_Node* x_rand = nullptr;
    if (goal_dis(goal_gen) > goal_sample_rate) { //如果生成的随机数大于目标采样概率，则生成随机点
        double randX = x_dis(goal_gen); //生成随机点X,Y
        double randY = y_dis(goal_gen);
        double randZ = z_dis(goal_gen);
        x_rand = new RRT_Node(Vector3d(randX,randY,randZ));//存储生成的随机位置

    }
    else 
    {   //否则目标点则为随机点
        x_rand = new RRT_Node(goal_data);
    }
    return x_rand;
}

RRT_Node* RRT::Near(RRT_Node* x_rand)
{
    int min_index = -1;
    double min_dis2 = std::numeric_limits<double>::max();
    for(int i = 0;i<node_list.size();++i)
    {
        double distance2 = std::pow(node_list[i]->data.x() - x_rand->data.x(), 2) +
                           std::pow(node_list[i]->data.y() - x_rand->data.y(), 2) +
                           std::pow(node_list[i]->data.z() - x_rand->data.z(), 2);
        if(distance2<min_dis2)
        {
            min_dis2 = distance2;
            min_index = i;
        }
    }
    return node_list[min_index];
}
void RRT::fixInMap(Eigen::Vector3d& new_node)
{
    new_node[0] = new_node[0] < 0? 0:new_node[0] > mapX? mapX:new_node[0];
    new_node[1] = new_node[1] < 0? 0:new_node[1] > mapY? mapY:new_node[1];
    new_node[2] = new_node[2] < 0? 0:new_node[2] > mapZ? mapZ:new_node[2];

}
RRT_Node* RRT::Step(RRT_Node* x_rand, RRT_Node* x_near)
{
    Eigen::Vector3d direction = x_rand->data - x_near->data;
    double dis_norm = direction.norm();
    double temp = step_size/dis_norm;
    // //RCLCPP_INFO(rclcpp::get_logger("rrt"),"temp:%f",temp);

    Eigen::Vector3d vec_step = temp*direction;
    //RCLCPP_INFO(rclcpp::get_logger("rrt"),"vec_step:%f,%f,%f",vec_step[0],vec_step[1],vec_step[2]);
    Eigen::Vector3d vec_new = x_near->data + vec_step;
    fixInMap(vec_new);
    RRT_Node* x_new = new RRT_Node(vec_new); 
    return x_new;
}

void RRT::AddNode(RRT_Node* x_new, RRT_Node* x_father)
{
    x_new->fatherPtr = x_father;
    node_list.push_back(x_new);
}

bool RRT::SuccessCheck(RRT_Node* x_new)
{
    Eigen::Vector3d dis(x_new->data.x()-goal_data.x(),
                        x_new->data.y()-goal_data.y(),
                        x_new->data.z()-goal_data.z());
    if(dis.squaredNorm()<=step_size)
    {
        return true;
    }
    else
    {
        return false;
    }
}
bool RRT::CollisionFree( RRT_Node* x_near,RRT_Node* x_new)
{
    //得到栅格坐标,非负
    
    Vector3d end = x_new->data * inv_resolution;
    Vector3d start = x_near->data * inv_resolution;
    RCLCPP_INFO(rclcpp::get_logger("CollisionFree"),"[%f,%f,%f],[%f,%f,%f]",start.x(),start.y(),start.z(),end.x(),end.y(),end.z());

    for(uint8_t i=0;i<3;++i)
    {
        start[i]<=0? min_double : start[i];
        end[i]<=0? min_double : end[i];
    }

    Vector3d d_vec = end-start;

    d_vec.x() = abs(d_vec.x())>min_double ? d_vec.x() : d_vec.x()>0? min_double:-min_double;
    d_vec.y() = abs(d_vec.y())>min_double ? d_vec.y() : d_vec.y()>0? min_double:-min_double;
    d_vec.z() = abs(d_vec.z())>min_double ? d_vec.z() : d_vec.z()>0? min_double:-min_double;

    Vector3i start_idx(std::floor(start.x()),std::floor(start.y()),std::floor(start.z()));
    Vector3i end_idx(std::floor(end.x()),std::floor(end.y()),std::floor(end.z()));
    Vector3i d_vec_i = end_idx-start_idx;
    //如果某个轴不动，把点移到中间，防止干扰
    for(uint8_t i=0;i<3;++i)
    {
        if(d_vec_i[i]==0)
        {
            start[i] = start_idx[i] + 0.5;
            end[i] = end_idx[i] + 0.5;
            d_vec[i] = min_double;
        }
    }
    Vector3d d_vec_sig(signum(d_vec.x()),signum(d_vec.y()),signum(d_vec.z()));
    
    //计算各轴比例
    double ykx = d_vec.y()/d_vec.x();
    double zkx = d_vec.z()/d_vec.x();

    double xky = d_vec.x()/d_vec.y();
    double zky = d_vec.z()/d_vec.y();

    double xkz = d_vec.x()/d_vec.z();
    double ykz = d_vec.y()/d_vec.z();

    while(true)
    {
        RCLCPP_INFO(rclcpp::get_logger("CollisionFreeWhile"),"[%d,%d,%d],[%d,%d,%d]",start_idx.x(),start_idx.y(),start_idx.z(),end_idx.x(),end_idx.y(),end_idx.z());
        //碰撞
        if (mapData[start_idx.x()*mapZ*mapY + start_idx.y()*mapZ + start_idx.z()]==1)
        {
            return false;
        }
        //成功到达
        if (start_idx==end_idx)
        {
            return true;
        }
        Vector3d bound_dis(res_distance(start.x(),d_vec.x()),res_distance(start.y(),d_vec.y()),res_distance(start.z(),d_vec.z()));
        Vector3d judge = bound_dis.array() / d_vec.array();
        if(judge.x()<judge.y() && judge.x()<judge.z())
        {
            Vector3d step(bound_dis.x(),ykx*bound_dis.x(),zkx*bound_dis.x());
            start = start + step + min_double*d_vec_sig;
        }
        else if(judge.y()<judge.x() && judge.y()<judge.z())
        {
            Vector3d step(xky*bound_dis.y(),bound_dis.y(),zky*bound_dis.y());
            start = start + step + min_double*d_vec_sig;
        }
        else
        {
            Vector3d step(xkz*bound_dis.z(),ykz*bound_dis.z(),bound_dis.z());
            start = start + step + min_double*d_vec_sig;
        }

        for(uint8_t i=0;i<3;++i)
        {
            start[i]<=0? min_double : start[i];
        }
        start_idx = Vector3i(std::floor(start.x()),std::floor(start.y()),std::floor(start.z()));
    }

    // double end_x = x_new->data.x() * inv_resolution;
    // double end_y = x_new->data.y() * inv_resolution;
    // double end_z = x_new->data.z() * inv_resolution;
    // double start_x = x_near->data.x() * inv_resolution;
    // double start_y = x_near->data.y() * inv_resolution;
    // double start_z = x_near->data.z() * inv_resolution;

    // //得到栅格索引
    // int endX = (int)std::floor(new_scale_x);
    // int endY = (int)std::floor(new_scale_y);
    // int endZ = (int)std::floor(new_scale_z);
    // int x = (int)std::floor(near_scale_x);
    // int y = (int)std::floor(near_scale_y);
    // int z = (int)std::floor(near_scale_z);

    // // Eigen::Vector3d direction(endX-x,endY-y,endZ-z);
    // // double maxDist = direction.squaredNorm();

    // // Break out direction vector.
    // int dx = endX - x;
    // int dy = endY - y;
    // int dz = endZ - z;

    // // Direction to increment x,y,z when stepping.
    // int stepX = signum(near_scale_x-new_scale_x);
    // int stepY = signum(near_scale_y-new_scale_y);
    // int stepZ = signum(near_scale_z-new_scale_z);

    // // See description above. The initial values depend on the fractional
    // // part of the origin.
    // double tMaxX = intbound(near_scale_x, dx);
    // double tMaxY = intbound(near_scale_y, dy);
    // double tMaxZ = intbound(near_scale_z, dz);

    // // The change in t when taking a step (always positive).
    // double tDeltaX = ((double)stepX) / dx;
    // double tDeltaY = ((double)stepY) / dy;
    // double tDeltaZ = ((double)stepZ) / dz;

    // double dist = 0;
    // while (true)
    // {
    //     //碰到障碍物
    //     if(x*mapZ*mapY + y*mapZ + z >= mapX*mapY*mapZ)
    //     {
    //         //RCLCPP_INFO(rclcpp::get_logger("rrt"),"G");
    //     }
    //     if (mapData[x*mapZ*mapY + y*mapZ + z]==1)
    //     {
    //         return false;
    //     }
    //     //成功到达
    //     if (x == endX && y == endY && z == endZ)
    //     {
    //         return true;
    //     }

    //     // tMaxX stores the t-value at which we cross a cube boundary along the
    //     // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
    //     // chooses the closest cube boundary. Only the first case of the four
    //     // has been commented in detail.
    //     if (tMaxX < tMaxY)
    //     {
    //         if (tMaxX < tMaxZ)
    //         {
    //             // Update which cube we are now in.
    //             x += stepX;
    //             // Adjust tMaxX to the next X-oriented boundary crossing.
    //             tMaxX += tDeltaX;
    //         }
    //         else
    //         {
    //             z += stepZ;
    //             tMaxZ += tDeltaZ;
    //         }
    //     }
    //     else
    //     {
    //         if (tMaxY < tMaxZ)
    //         {
    //             y += stepY;
    //             tMaxY += tDeltaY;
    //         }
    //         else
    //         {
    //             z += stepZ;
    //             tMaxZ += tDeltaZ;
    //         }
    //     }
    // }
}

std::vector<Vector3d> RRT::Planning()
{
    RRT_Node* x_rand  = nullptr;
    RRT_Node* x_near  = nullptr;
    RRT_Node* x_new  = nullptr;
    std::vector<Vector3d> output;
    if(SuccessCheck(node_list[0]))
    {
        output.push_back(map2rviz(node_list[0]->data));
        return output;
    }
    int count = 0;
    while(true)
    {
        count++;
        if(count>1500)
        {
            RCLCPP_INFO(rclcpp::get_logger("rrt"),"too much");
            break;
        }
        
        x_rand = Sample();
        RCLCPP_INFO(rclcpp::get_logger("rrt"),"Sample:%f,%f,%f",x_rand->data[0],x_rand->data[1],x_rand->data[2]);
        x_near = Near(x_rand);
        RCLCPP_INFO(rclcpp::get_logger("rrt"),"Near:%f,%f,%f",x_near->data[0],x_near->data[1],x_near->data[2]);
        x_new = Step(x_rand,x_near);
        RCLCPP_INFO(rclcpp::get_logger("rrt"),"Step:%f,%f,%f",x_new->data[0],x_new->data[1],x_new->data[2]);
        if(CollisionFree(x_near,x_new))
        {
            RCLCPP_INFO(rclcpp::get_logger("rrt"),"true");

            AddNode(x_new,x_near);
            delete x_rand;
            if(SuccessCheck(x_new))
            {
                break;
            }
        }
        else
        {
            //RCLCPP_INFO(rclcpp::get_logger("rrt"),"false");

            delete x_rand;
            delete x_new;

        }
        //RCLCPP_INFO(rclcpp::get_logger("rrt"),"CollisionFree");
    }

    RRT_Node* x_i = x_new;
    //RCLCPP_INFO(rclcpp::get_logger("rrt"),"SuccessCheck:%p",x_i);

    while(x_i->fatherPtr!=nullptr)
    {
        //RCLCPP_INFO(rclcpp::get_logger("rrt"),"SuccessCheck:%f,%f,%f",x_i->data[0],x_i->data[1],x_i->data[2]);
        output.push_back(map2rviz(x_i->data));
        x_i = x_i->fatherPtr;
    }
    output.push_back(map2rviz(x_i->data));//起点
    return output;
}

Vector3d RRT::map2rviz(const Vector3d& index) const
{
    Vector3d pt;
    pt = index + origin;
    return pt;
}

Vector3d RRT::rviz2map(const Vector3d& pt) const
{
    Vector3d pt_;
    pt_ <<  min( max( pt(0) - origin(0), 0.0), mapX*resolution),
           min( max( pt(1) - origin(1), 0.0), mapY*resolution),
           min( max( pt(2) - origin(2), 0.0), mapZ*resolution);                  
  
    return pt_;
}
}