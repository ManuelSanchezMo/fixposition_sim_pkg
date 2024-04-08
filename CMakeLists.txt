cmake_minimum_required(VERSION 3.8)
project(fixposition_sensor_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED )

link_directories(${gazebo_dev_LIBRARY_DIRS})


# gazebo_ros_j8_wheels_speed
add_library(gazebo_ros_fixposition_plugin SHARED
  src/gazebo_ros_fixposition.cpp
)



target_include_directories(gazebo_ros_fixposition_plugin PUBLIC include)
ament_target_dependencies(gazebo_ros_fixposition_plugin
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  "geometry_msgs"
  "sensor_msgs"
)

target_include_directories(gazebo_ros_fixposition_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_libraries(gazebo_ros_fixposition_plugin)
if(ENABLE_PROFILER)
  target_include_directories(gazebo_ros_fixposition_plugin PUBLIC ${ignition-common3_INCLUDE_DIRS})
  target_link_libraries(gazebo_ros_fixposition_plugin ${ignition-common3_LIBRARIES})
endif()

# gazebo_ros_gps_sensor_mod
add_library(gazebo_ros_gps_sensor_mod SHARED
  src/gazebo_ros_fixposition_gps_sensor.cpp)
  
  target_include_directories(gazebo_ros_gps_sensor_mod
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
    
target_include_directories(gazebo_ros_gps_sensor_mod PUBLIC include)
ament_target_dependencies(gazebo_ros_gps_sensor_mod
"gazebo_ros"
"sensor_msgs"
"geometry_msgs"
"gazebo_dev"
)

ament_export_libraries(gazebo_ros_gps_sensor_mod)
if(ENABLE_PROFILER)
  target_include_directories(gazebo_ros_gps_sensor_mod PUBLIC ${ignition-common3_INCLUDE_DIRS})
  target_link_libraries(gazebo_ros_gps_sensor_mod ${ignition-common3_LIBRARIES})
endif()

# gazebo_ros_imu_sensor_mod
add_library(gazebo_ros_imu_sensor_mod SHARED
  src/gazebo_ros_fixposition_imu_sensor.cpp)
  
  target_include_directories(gazebo_ros_imu_sensor_mod
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
    
target_include_directories(gazebo_ros_imu_sensor_mod PUBLIC include)
ament_target_dependencies(gazebo_ros_imu_sensor_mod
  "gazebo_ros"
  "sensor_msgs"
  "gazebo_dev"
  "tf2_geometry_msgs"
)
target_link_libraries(gazebo_ros_imu_sensor_mod
  ImuSensorPlugin
)
ament_export_libraries(gazebo_ros_imu_sensor_mod)
if(ENABLE_PROFILER)
  target_include_directories(gazebo_ros_imu_sensor_mod PUBLIC ${ignition-common3_INCLUDE_DIRS})
  target_link_libraries(gazebo_ros_imu_sensor_mod ${ignition-common3_LIBRARIES})
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
gazebo_ros_fixposition_plugin
gazebo_ros_imu_sensor_mod
gazebo_ros_gps_sensor_mod
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF/Xacro files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

