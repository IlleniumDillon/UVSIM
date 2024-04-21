#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_SOKOBAN_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_SOKOBAN_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosSokobanPrivate;

class GazeboRosSokoban : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosSokoban();

  /// Destructor
  ~GazeboRosSokoban();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosSokobanPrivate> impl_;
};
}  // namespace gazebo_plugins
#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_JOINT_POSE_TRAJECTORY_HPP_
