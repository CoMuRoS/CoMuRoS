cmake_minimum_required(VERSION 3.8)
project(yahboom_rosmaster_description)

# Check if the compiler being used is GNU's C++ compiler (g++) or Clang.
# Add compiler flags for all targets that will be defined later in the
# CMakeLists file. These flags enable extra warnings to help catch
# potential issues in the code.
# Add options to the compilation process
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Locate and configure packages required by the project.
find_package(ament_cmake REQUIRED)
find_package(urdf_tutorial REQUIRED)

# Copy necessary files to designated locations in the project
install (
  DIRECTORY config launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)


ament_environment_hooks(
     "${CMAKE_CURRENT_SOURCE_DIR}/env/ign_resource_path.dsv.in"
   )

ament_package()
