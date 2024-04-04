#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_MODELSTATE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_MODELSTATE_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosModelStatePrivate;

class GazeboRosModelState : public gazebo::WorldPlugin
{
public:
  /// Constructor
  GazeboRosModelState();

  /// Destructor
  ~GazeboRosModelState();

protected:
  // Documentation inherited
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosModelStatePrivate> impl_;
};

}

#endif