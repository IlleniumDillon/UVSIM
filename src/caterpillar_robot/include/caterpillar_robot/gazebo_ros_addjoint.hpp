#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_ADDJOINT_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_ADDJOINT_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosAddJointPrivate;

class GazeboRosAddJoint : public gazebo::WorldPlugin
{
public:
  /// Constructor
  GazeboRosAddJoint();

  /// Destructor
  ~GazeboRosAddJoint();

protected:
  // Documentation inherited
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosAddJointPrivate> impl_;
};

}

#endif