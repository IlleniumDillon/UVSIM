#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros_modelstate.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <simbridge/msg/model_state.h>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosModelStatePrivate
{

};

GazeboRosModelState::GazeboRosModelState():impl_(std::make_unique<GazeboRosModelStatePrivate>())
{
}

GazeboRosModelState::~GazeboRosModelState()
{
}
void GazeboRosModelState::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosModelState)

}