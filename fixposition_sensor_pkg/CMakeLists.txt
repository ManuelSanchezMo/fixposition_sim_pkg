cmake_minimum_required(VERSION 3.8)
project(fixposition_sensor_pkg)

# Enable C++14 (or higher) standards
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-g)

# Compiler flags for better warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Find Package Dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fixposition_msgs REQUIRED)  # Custom message package
find_package(Eigen3 REQUIRED)             # For Eigen types used in helper class

# -----------------------------
# Export Runtime Dependencies
# -----------------------------
# Ensure that any package depending on this package also depends on these
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(fixposition_msgs)

# -----------------------------
# Add GazeboFixpositionHelper Library
# -----------------------------
add_library(gazebo_fixposition_helper SHARED src/gazebo_fixposition_helper.cpp)
target_include_directories(gazebo_fixposition_helper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gazebo_fixposition_helper
  geometry_msgs
  sensor_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  gazebo_dev
  gazebo_ros
  nav_msgs
  Eigen3
  fixposition_msgs
  std_srvs
)

# -----------------------------
# Add Main Plugin Libraries
# -----------------------------
# 1. GazeboFixpositionPlugin
add_library(gazebo_ros_fixposition_plugin SHARED src/gazebo_ros_fixposition.cpp)
target_include_directories(gazebo_ros_fixposition_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gazebo_ros_fixposition_plugin
  gazebo_dev
  gazebo_ros
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  sensor_msgs
  fixposition_msgs
)
# Link the helper library to the main plugin
target_link_libraries(gazebo_ros_fixposition_plugin
  gazebo_fixposition_helper
)

# 2. GazeboROSGPSSensorMod
add_library(gazebo_ros_gps_sensor_mod SHARED src/gazebo_ros_fixposition_gps_sensor.cpp)
target_include_directories(gazebo_ros_gps_sensor_mod PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gazebo_ros_gps_sensor_mod
  gazebo_ros
  sensor_msgs
  geometry_msgs
  gazebo_dev
  nav_msgs
  tf2_ros
  fixposition_msgs
)
# Link the helper library to the GPS sensor plugin
target_link_libraries(gazebo_ros_gps_sensor_mod
  gazebo_fixposition_helper
)

# 3. GazeboROSIMUSensorMod
add_library(gazebo_ros_imu_sensor_mod SHARED src/gazebo_ros_fixposition_imu_sensor.cpp)
target_include_directories(gazebo_ros_imu_sensor_mod PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gazebo_ros_imu_sensor_mod
  gazebo_ros
  sensor_msgs
  gazebo_dev
  tf2_geometry_msgs
  fixposition_msgs
)
# Link the helper library and Gazebo libraries to the IMU sensor plugin
target_link_libraries(gazebo_ros_imu_sensor_mod
  gazebo_fixposition_helper
  ${GAZEBO_LIBRARIES}
)

# -----------------------------
# Install Targets
# -----------------------------
install(TARGETS
  gazebo_fixposition_helper
  gazebo_ros_fixposition_plugin
  gazebo_ros_gps_sensor_mod
  gazebo_ros_imu_sensor_mod
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# -----------------------------
# Install Header Files
# -----------------------------
# Ensure that header files are accessible to other packages
install(DIRECTORY include/
  DESTINATION include/
)

# -----------------------------
# Install URDF/Xacro and Meshes
# -----------------------------
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME}/)

# -----------------------------
# Finalize the Package
# -----------------------------
ament_package()

