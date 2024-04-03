#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros_servojoint.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosServoJointPrivate
{
public:
    void OnUpdate(const gazebo::common::UpdateInfo & info);
    void SetJointState(std_msgs::msg::Float64::SharedPtr msg);
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
    gazebo::physics::ModelPtr model_;
    gazebo::physics::WorldPtr world_;
    gazebo::physics::LinkPtr reference_link_;
    gazebo::physics::JointPtr joint_;
    double cmd_;
    double update_period_;
    gazebo::common::Time last_update_time_;
    gazebo::common::Time trajectory_start_time_;
    std::mutex lock_;
    unsigned int trajectory_index_;
    bool new_cmds_ = false;
    gazebo::event::ConnectionPtr update_connection_;
};

GazeboRosServoJoint::GazeboRosServoJoint()
: impl_(std::make_unique<GazeboRosServoJointPrivate>())
{
}

GazeboRosServoJoint::~GazeboRosServoJoint()
{
}

void GazeboRosServoJoint::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    impl_->model_ = model;
    impl_->world_ = model->GetWorld();
    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
    const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();
    auto update_rate = sdf->Get<double>("update_rate", 100.0).first;
    if (update_rate > 0.0) 
    {
        impl_->update_period_ = 1.0 / update_rate;
    } 
    else 
    {
        impl_->update_period_ = 0.0;
    }
    for (auto left_joint_elem = sdf->GetElement("joints"); left_joint_elem != nullptr;
    left_joint_elem = left_joint_elem->GetNextElement("joints"))
    {
        auto joint_name = left_joint_elem->Get<std::string>();
        auto joint = impl_->model_->GetJoint(joint_name);
        if (!joint) 
        {
            RCLCPP_ERROR(
                impl_->ros_node_->get_logger(),
                "Joint [%s] not found, plugin will not work.", joint_name.c_str());
            impl_->ros_node_.reset();
            return;
        }
        impl_->joint_ = joint;
    }
    
    impl_->last_update_time_ = impl_->world_->SimTime();
    impl_->sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float64>(
    "set_servojoint", qos.get_subscription_qos("set_servojoint", rclcpp::QoS(1)),
    std::bind(
      &GazeboRosServoJointPrivate::SetJointState,
      impl_.get(), std::placeholders::_1));
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosServoJointPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}
void GazeboRosServoJointPrivate::OnUpdate(const gazebo::common::UpdateInfo &info)
{
    double maxspeed = 1;
    gazebo::common::Time current_time = info.simTime;
    if (current_time < last_update_time_) {
        RCLCPP_INFO(ros_node_->get_logger(), "Negative sim time difference detected.");
        last_update_time_ = current_time;
    }
    double seconds_since_last_update = (current_time - last_update_time_).Double();
    if (seconds_since_last_update < update_period_) {
        return;
    }
    last_update_time_ = current_time;
    if(!new_cmds_) return;
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE("GazeboRosServoJointPrivate::OnUpdate");
    IGN_PROFILE_BEGIN("update");
#endif
    std::lock_guard<std::mutex> scoped_lock(lock_);
    double cur = joint_->Position(0);
    double err = cmd_ - cur;
    double maxAng = maxspeed * seconds_since_last_update;
    double delta = abs(err) > maxAng ? err/abs(err)*maxAng : err;
    joint_->SetPosition(0,cur + delta);
    new_cmds_ = false;

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

void GazeboRosServoJointPrivate::SetJointState(std_msgs::msg::Float64::SharedPtr msg)
{
    cmd_ = msg->data;
    new_cmds_ = true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosServoJoint)
}