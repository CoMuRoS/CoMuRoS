#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/Light.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/SurfaceParams.hh>
#include <gazebo/physics/PhysicsEngine.hh>

#include <gazebo_ros/node.hpp>
#include <memory>

#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"

#include "mu_runtime/gazebo_runtime_mu.hpp"   // INCLUDE HEADER FILE.
#include <gazebo_mu_interfaces/srv/set_friction.hpp>        // INCLUDE ROS2 SERVICE.

namespace gazebo_ros
{

class GazeboRuntimeMuPrivate
{
public:

  // ATTACH (ROS2 service):
  void SetFriction(
    gazebo_mu_interfaces::srv::SetFriction::Request::SharedPtr req,
    gazebo_mu_interfaces::srv::SetFriction::Response::SharedPtr res);

  // World pointer from Gazebo.
  gazebo::physics::WorldPtr world_;

  /// ROS node for communication, managed by gazebo_ros.
  gazebo_ros::Node::SharedPtr ros_node_2;

  // ROS services to handle requests for attach/detach.
  rclcpp::Service<gazebo_mu_interfaces::srv::SetFriction>::SharedPtr set_friction_service;

};

GazeboRuntimeMu::GazeboRuntimeMu()
: impl_(std::make_unique<GazeboRuntimeMuPrivate>())
{
}

GazeboRuntimeMu::~GazeboRuntimeMu()
{
}

void GazeboRuntimeMu::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  
  // Gazebo WORLD:
  impl_->world_ = _world;

  // ROS2 NODE:
  impl_->ros_node_2 = gazebo_ros::Node::Get(_sdf);

  // ROS2 SERVICE SERVERS:
  impl_->set_friction_service =
    impl_->ros_node_2->create_service<gazebo_mu_interfaces::srv::SetFriction>(
    "set_friction", std::bind(
      &GazeboRuntimeMuPrivate::SetFriction, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

}

void GazeboRuntimeMuPrivate::SetFriction(
  std::shared_ptr<gazebo_mu_interfaces::srv::SetFriction::Request> req,
  std::shared_ptr<gazebo_mu_interfaces::srv::SetFriction::Response> res)
{
  // Retrieve the model
  auto model = world_->ModelByName(req->model_name);
  if (!model) {
    res->success = false;
    res->message = "Model not found.";
    return;
  }

  // Retrieve the link
  auto link = model->GetLink(req->link_name);
  if (!link) {
    res->message = "Link not found.";
    res->success = false;
    return;
  }

  // Retrieve the collision element
  auto collisions = link->GetCollisions();
  if (collisions.empty()) {
    res->success = false;
    res->message = "No collision elements found in the link.";
    return;
  }
  auto collision = collisions[0];  // Get the first one

  // Convert mu1 and mu2 from string to double
  try {
    double mu1 = std::stod(req->mu1);
    double mu2 = std::stod(req->mu2);

    // Set the friction parameters
    auto surface = collision->GetSurface();
    auto friction = surface->FrictionPyramid();
    friction->SetMuPrimary(mu1);
    friction->SetMuSecondary(mu2);

    res->success = true;
    res->message = "Friction parameters updated successfully.";
  } catch (const std::invalid_argument& e) {
    res->success = false;
    res->message = "Invalid argument: unable to convert mu1 or mu2 to double.";
  } catch (const std::out_of_range& e) {
    res->success = false;
    res->message = "Out of range: mu1 or mu2 is too large to convert to double.";
  }
}

GZ_REGISTER_WORLD_PLUGIN(GazeboRuntimeMu)

}