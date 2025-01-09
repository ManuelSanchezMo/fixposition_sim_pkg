cmake_minimum_required(VERSION 3.8)
project(fixposition_msgs)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate ROS2 interfaces for custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GNSSANT.msg"
  "msg/GNSSCORR.msg"
  "msg/IMUBIAS.msg"
  "msg/ODOMETRY.msg"
  "msg/ODOMSTATUS.msg"
  "msg/ODOMSH.msg"
  "msg/ODOMENU.msg"
  "msg/LLH.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# Export dependencies for message interfaces
ament_export_dependencies(rosidl_default_runtime)
ament_package()

