cmake_minimum_required(VERSION 3.8)
project(multi_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install (
  DIRECTORY launch worlds rviz models
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
     "${CMAKE_CURRENT_SOURCE_DIR}/env/ign_resource_path.dsv.in"
   )

ament_package()
