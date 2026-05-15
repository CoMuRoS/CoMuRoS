cmake_minimum_required(VERSION 3.8)
project(mu_runtime)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Find required packages:
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_package(gazebo_mu_interfaces REQUIRED)

# /include:
include_directories(include)

# GAZEBO LINK ATTACHER:

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(gazebo_runtime_mu SHARED
  src/gazebo_runtime_mu.cpp
)
target_link_libraries(gazebo_runtime_mu ${GAZEBO_LIBRARIES})
ament_target_dependencies(gazebo_runtime_mu
  "rclcpp"
  "gazebo_dev"
  "gazebo_msgs"
  "gazebo_ros"
  "gazebo_mu_interfaces"
)
ament_export_libraries(gazebo_runtime_mu)
install(
  TARGETS
  gazebo_runtime_mu
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


ament_package()
