#ifndef GAZEBO_ROS_FIXPOSITION_IMU_SENSOR_HPP
#define GAZEBO_ROS_FIXPOSITION_IMU_SENSOR_HPP

#include <memory>
#include <gazebo/plugins/ImuSensorPlugin.hh>

namespace gazebo_plugins {

// Forward declaration of private implementation class
class GazeboRosImuSensorPrivate;

class GazeboRosImuSensor : public gazebo::SensorPlugin {
public:
  GazeboRosImuSensor();
  ~GazeboRosImuSensor() override;

  // Override the Load method to initialize the plugin
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

private:
  // Unique pointer to the private implementation
  std::unique_ptr<GazeboRosImuSensorPrivate> impl_;
};

}  // namespace gazebo_plugins

#endif  // GAZEBO_ROS_FIXPOSITION_IMU_SENSOR_HPP

