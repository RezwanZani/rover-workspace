cmake_minimum_required(VERSION 3.8)
project(rover_simplified)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(serial_receiver_node src/serial_receiver_node.cpp)
ament_target_dependencies(serial_receiver_node
  rclcpp
  geometry_msgs
)

# Install directories to the share folder
install(TARGETS
  serial_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

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