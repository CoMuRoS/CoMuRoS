cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

# Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hello_moveit_interfaces REQUIRED)  # Dependency for custom service

# Add executables
add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(cartesian_planner src/cartesian_planner.cpp)
add_executable(pose_service_node src/pose_service.cpp)
add_executable(gripper_control_service src/gripper_control_service.cpp)

# Link dependencies for hello_moveit
ament_target_dependencies(
  hello_moveit
  moveit_ros_planning_interface
  rclcpp
)

# Link dependencies for cartesian_planner
ament_target_dependencies(
  cartesian_planner
  moveit_ros_planning_interface
  rclcpp
)



# Link dependencies for pose_service_node
ament_target_dependencies(
  pose_service_node
  moveit_ros_planning_interface
  rclcpp
  hello_moveit_interfaces
)

ament_target_dependencies(gripper_control_service rclcpp moveit_ros_planning_interface std_srvs)

# Set C++ standard for all targets
target_compile_features(hello_moveit PUBLIC cxx_std_17)
target_compile_features(cartesian_planner PUBLIC cxx_std_17)
target_compile_features(pose_service_node PUBLIC cxx_std_17)

# Install executables
install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS cartesian_planner
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS pose_service_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS gripper_control_service DESTINATION lib/${PROJECT_NAME})

# Testing (Optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linting checks if required
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
