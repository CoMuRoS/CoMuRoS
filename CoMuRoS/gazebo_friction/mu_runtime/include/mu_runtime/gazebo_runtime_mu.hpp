 
#ifndef GAZEBO_RUNTIME_MU_HPP_
#define GAZEBO_RUNTIME_MU_HPP_

#include <gazebo/common/Plugin.hh>
#include <memory>

namespace gazebo_ros
{

class GazeboRuntimeMuPrivate;

class GazeboRuntimeMu : public gazebo::WorldPlugin
{
public:
  
  // Constructor:
  GazeboRuntimeMu();

  // Destructor:
  virtual ~GazeboRuntimeMu();

  // LOAD plugin:
  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;


private:

  std::unique_ptr<GazeboRuntimeMuPrivate> impl_;

};

}  // namespace gazebo_ros

#endif  // GAZEBO_RUNTIME_MU_HPP_