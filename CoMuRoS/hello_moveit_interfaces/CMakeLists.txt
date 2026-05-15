cmake_minimum_required(VERSION 3.8)
project(hello_moveit_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate the service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPose.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
