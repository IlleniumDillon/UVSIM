#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros_sokoban.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <simbridge/msg/sokoban_task.hpp>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosSokobanPrivate
{
public:
    void OnUpdate(const gazebo::common::UpdateInfo & info);
    void GetTasks(simbridge::msg::SokobanTask::SharedPtr msg);
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<simbridge::msg::SokobanTask>::SharedPtr sub_;
    gazebo::physics::ModelPtr model_;
    gazebo::physics::WorldPtr world_;
    gazebo::physics::JointPtr joint_arm;
    gazebo::physics::JointPtr joint_hand;
    double target_arm = -2;
    double target_hand = 2;
    simbridge::msg::SokobanTask task;
    bool initialized_ = false;
    int task_id_;
    uint8_t lastAction = 0;
    double update_period_;
    gazebo::common::Time last_update_time_;
    gazebo::common::Time trajectory_start_time_;
    std::mutex lock_;
    unsigned int trajectory_index_;
    gazebo::event::ConnectionPtr update_connection_;
};
void GazeboRosSokobanPrivate::OnUpdate(const gazebo::common::UpdateInfo &info)
{
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
    if (!initialized_) {
        model_->SetLinearVel({0,0,0});
        model_->SetAngularVel({0,0,0});
        joint_arm->SetPosition(0, -2);
        joint_hand->SetPosition(0, 2);
        return;
    }
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE("GazeboRosSokobanPrivate::OnUpdate");
    IGN_PROFILE_BEGIN("update");
#endif
    std::lock_guard<std::mutex> scoped_lock(lock_);
    ignition::math::Pose3d curPose = model_->RelativePose();
    double curYaw = curPose.Yaw();
    ignition::math::Vector3d curPos = curPose.Pos();

    if (task_id_ >= task.action.size()) 
    {
        initialized_ = false;
        task_id_ = -1;
        model_->SetLinearVel({0,0,0});
        model_->SetAngularVel({0,0,0});
        return;
    }
    //target position
    ignition::math::Vector3d targetPos{task.task_points[task_id_].x, 
        task.task_points[task_id_].y, 
        task.task_points[task_id_].z};
    //first get yaw err
    double targetYaw;
    if(task_id_ == 0)
    {
        targetYaw = atan2(targetPos.Y() - curPos.Y(), targetPos.X() - curPos.X());
    }
    else
    {
        targetYaw = atan2(targetPos.Y() - task.task_points[task_id_-1].y, targetPos.X() - task.task_points[task_id_-1].x);
    }
    //RCLCPP_INFO(ros_node_->get_logger(), "targetYaw: %f curYaw: %f", targetYaw, curYaw);
    double diffYaw = targetYaw - curYaw;
    if (diffYaw > M_PI) 
    {
        diffYaw -= 2 * M_PI;
    } 
    else if (diffYaw < -M_PI) 
    {
        diffYaw += 2 * M_PI;
    }
    //then get position err
    double diffPos = (targetPos - curPos).Length();
    //set velocity
    double linearVel = diffPos*2 > 0.3 ? 0.3 : diffPos*2;
    double angularVel = fabs(diffYaw)*3 > 0.3 ? diffYaw/fabs(diffYaw)*0.3 : diffYaw*3;
    if(fabs(diffYaw) >= 0.05)
    {
        model_->SetLinearVel({0,0,0});
        model_->SetAngularVel({0,0,angularVel});
        target_arm = -2;
        target_hand = 2;
    }
    else
    {
        double errX = targetPos.X() - curPos.X();
        double errY = targetPos.Y() - curPos.Y();
        double errZ = targetPos.Z() - curPos.Z();

        double deltaX = fabs(errX)*3 > 0.3 ? errX/fabs(errX)*0.3 : errX*3;
        double deltaY = fabs(errY)*3 > 0.3 ? errY/fabs(errY)*0.3 : errY*3;
        model_->SetLinearVel( {deltaX, deltaY, 0} );
        model_->SetAngularVel({0,0,0});
        if (task.action[task_id_] == 2) 
        {
            target_arm = -2;
            target_hand = 2;
        }
        else
        {
            target_arm = 0;
            target_hand = 0;
        }
    }
    if (diffPos < 0.08 && fabs(diffYaw) < 0.05) 
    {
        //model_->SetRelativePose({targetPos, {0,0,targetYaw}});
        task_id_++;
    }
    double cur = joint_arm->Position(0);
    double err = target_arm - cur;
    double maxAng = seconds_since_last_update;
    double delta = abs(err) > maxAng ? err/abs(err)*maxAng : err;
    joint_arm->SetPosition(0,cur + delta);
    //RCLCPP_INFO(ros_node_->get_logger(), "target_arm: %f cur_arm: %f", target_arm, cur);
    cur = joint_hand->Position(0);
    err = target_hand - cur;
    delta = abs(err) > maxAng ? err/abs(err)*maxAng : err;
    joint_hand->SetPosition(0,cur + delta);
    //uint8_t action = task.action[task_id_];

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

void GazeboRosSokobanPrivate::GetTasks(simbridge::msg::SokobanTask::SharedPtr msg)
{
    RCLCPP_INFO(ros_node_->get_logger(), "Received task");
    task = *msg;
    initialized_ = true;
    task_id_ = 0;
}

GazeboRosSokoban::GazeboRosSokoban(): impl_(std::make_unique<GazeboRosSokobanPrivate>())
{
}

GazeboRosSokoban::~GazeboRosSokoban()
{}

void GazeboRosSokoban::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    impl_->model_ = model;
    impl_->world_ = model->GetWorld();
    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
    const gazebo_ros::QoS &qos = impl_->ros_node_->get_qos();
    auto update_rate = sdf->Get<double>("update_rate", 100.0).first;
    if (update_rate > 0.0)
    {
        impl_->update_period_ = 1.0 / update_rate;
    }
    else
    {
        impl_->update_period_ = 0.0;
    }
    auto joint_arm_name = sdf->Get<std::string>("joint_arm");
    auto joint_hand_name = sdf->Get<std::string>("joint_hand");
    impl_->joint_arm = impl_->model_->GetJoint(joint_arm_name);
    impl_->joint_hand = impl_->model_->GetJoint(joint_hand_name);
    if(!impl_->joint_arm || !impl_->joint_hand)
    {
        RCLCPP_ERROR(
            impl_->ros_node_->get_logger(),
            "Joint [%s] or [%s] not found, plugin will not work.", joint_arm_name.c_str(), joint_hand_name.c_str());
        impl_->ros_node_.reset();
        return;
    }
    impl_->sub_ = impl_->ros_node_->create_subscription<simbridge::msg::SokobanTask>(
        "sokoban_task", qos.get_subscription_qos("sokoban_task", rclcpp::QoS(1)),
        std::bind(&GazeboRosSokobanPrivate::GetTasks, impl_.get(), std::placeholders::_1));
    impl_->last_update_time_ = impl_->world_->SimTime();
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRosSokobanPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosSokoban)
} // namespace gazebo_plugins