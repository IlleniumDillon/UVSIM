#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_SERVOJOINT_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_SERVOJOINT_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosServoJointPrivate;

class GazeboRosServoJoint : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosServoJoint();

  /// Destructor
  ~GazeboRosServoJoint();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosServoJointPrivate> impl_;
};
}  // namespace gazebo_plugins
#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_JOINT_POSE_TRAJECTORY_HPP_
