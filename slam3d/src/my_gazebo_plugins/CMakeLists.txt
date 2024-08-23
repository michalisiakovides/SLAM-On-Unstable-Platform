cmake_minimum_required(VERSION 3.5)
project(constant_velocity_plugin)

# Find the necessary packages
find_package(gazebo REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

# Link directories
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Add the plugin library
add_library(ConstantVelocityPlugin SHARED src/constant_velocity_plugin.cpp)

# Link the libraries
target_link_libraries(ConstantVelocityPlugin
  ${GAZEBO_LIBRARIES}
  ${IGNITION-MATH_LIBRARIES}
)

# Specify the dependencies for ament
ament_target_dependencies(ConstantVelocityPlugin
  rclcpp
  geometry_msgs
)

# Install the plugin
install(
  TARGETS ConstantVelocityPlugin
  LIBRARY DESTINATION lib
)

