#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros_addjoint.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <simbridge/msg/model_state.hpp>
#include <simbridge/msg/add_joint.hpp>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosAddJointPrivate
{
public:
    void addJointCallback(const simbridge::msg::AddJoint::SharedPtr msg);
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<simbridge::msg::AddJoint>::SharedPtr sub_;
    gazebo::physics::ModelPtr model_;
    gazebo::physics::WorldPtr world_;
};

GazeboRosAddJoint::GazeboRosAddJoint():impl_(std::make_unique<GazeboRosAddJointPrivate>())
{
}

GazeboRosAddJoint::~GazeboRosAddJoint()
{
}
void GazeboRosAddJoint::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
{
    impl_->world_ = world;
    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
    const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();
    impl_->sub_ = impl_->ros_node_->create_subscription<simbridge::msg::AddJoint>(
        "/add_joint",1,std::bind(&GazeboRosAddJointPrivate::addJointCallback,impl_.get(),std::placeholders::_1)
    );
}

void GazeboRosAddJointPrivate::addJointCallback(const simbridge::msg::AddJoint::SharedPtr msg)
{
    gazebo::physics::ModelPtr model1 = world_->ModelByName(msg->model1);
    if(model1 == NULL)
    {
        RCLCPP_ERROR(ros_node_->get_logger(),"no model named: %s", msg->model1.c_str());
    }
    gazebo::physics::Link_V linkList1 = model1->GetLinks();
    for(int i = 0; i < linkList1.size(); i++)
    {
        RCLCPP_INFO(ros_node_->get_logger(),"link named: %s", linkList1.at(i)->GetName().c_str());
    }
    gazebo::physics::ModelPtr model2 = world_->ModelByName(msg->model2);
    if(model2 == NULL)
    {
        RCLCPP_ERROR(ros_node_->get_logger(),"no model named: %s", msg->model2.c_str());
    }
    gazebo::physics::Link_V linkList2 = model2->GetLinks();
    for(int i = 0; i < linkList2.size(); i++)
    {
        RCLCPP_INFO(ros_node_->get_logger(),"link named: %s", linkList2.at(i)->GetName().c_str());
    }
    model1->CreateJoint("temp","fixed",model2->GetLink("robot0_arm_hand_link"),model1->GetLink("link_0"));
}

GZ_REGISTER_WORLD_PLUGIN(GazeboRosAddJoint)

}