cmake_minimum_required(VERSION 3.8)
project(rover_simplified)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)

# Install directories to the share folder
# This ensures ROS 2 can find your URDF, 3D meshes, and launch files during runtime.
install(
  DIRECTORY 
    urdf 
    worlds
    launch
  DESTINATION share/${PROJECT_NAME}
)

# Initialize the ament build system (must be at the end of the file)
ament_package()