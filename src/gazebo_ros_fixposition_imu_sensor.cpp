#include "fixposition_sensor_pkg/gazebo_ros_fixposition_imu_sensor.hpp"

#include <memory>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <fixposition_msgs/msg/imubias.hpp>
#include <rclcpp/logging.hpp>
#include "fixposition_sensor_pkg/NoiseGenerator.hpp"

namespace gazebo_plugins {

class GazeboRosImuSensorPrivate {
public:
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Publisher<fixposition_msgs::msg::IMUBIAS>::SharedPtr bias_pub_;
  sensor_msgs::msg::Imu::SharedPtr msg_;
  fixposition_msgs::msg::IMUBIAS::SharedPtr bias_msg_;
  gazebo::sensors::ImuSensorPtr sensor_;
  gazebo::event::ConnectionPtr sensor_update_event_;
  NoiseGenerator linear_acceleration_bias_;
  NoiseGenerator angular_velocity_bias_;
  double acc_bias_covariance_ = 0.0;
  double gyr_bias_covariance_ = 0.0;

  void OnUpdate();
};

GazeboRosImuSensor::GazeboRosImuSensor()
    : impl_(std::make_unique<GazeboRosImuSensorPrivate>()) {}

GazeboRosImuSensor::~GazeboRosImuSensor() {}

void GazeboRosImuSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  const auto &qos = impl_->ros_node_->get_qos();

  impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(_sensor);

  if (!impl_->sensor_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Parent is not an IMU sensor. Exiting.");
    return;
  }

  double bias_mean = _sdf->Get<double>("bias_mean", 0.0).first;
  double bias_std_dev = _sdf->Get<double>("bias_std_dev", 0.01).first;
  double drift_rate = _sdf->Get<double>("drift_rate", 0.0001).first;

  impl_->acc_bias_covariance_ = _sdf->Get<double>("acc_bias_covariance", 0.001).first;
  impl_->gyr_bias_covariance_ = _sdf->Get<double>("gyr_bias_covariance", 0.0005).first;

  impl_->linear_acceleration_bias_.SetBiasDriftParameters(bias_mean, drift_rate);
  impl_->linear_acceleration_bias_.SetGaussianParameters(bias_mean, bias_std_dev);
  impl_->angular_velocity_bias_.SetBiasDriftParameters(bias_mean, drift_rate);
  impl_->angular_velocity_bias_.SetGaussianParameters(bias_mean, bias_std_dev);

  RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "IMU plugin configured: mean=%.3f, std_dev=%.3f, drift_rate=%.3f",
      bias_mean, bias_std_dev, drift_rate);

  impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Imu>(
      "~/out", qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().reliable()));

  impl_->bias_pub_ = impl_->ros_node_->create_publisher<fixposition_msgs::msg::IMUBIAS>(
      "/fixposition/fpa/imubias", qos.get_publisher_qos("/fixposition/fpa/imubias", rclcpp::QoS(1).reliable()));

  impl_->msg_ = std::make_shared<sensor_msgs::msg::Imu>();
  impl_->bias_msg_ = std::make_shared<fixposition_msgs::msg::IMUBIAS>();

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
      std::bind(&GazeboRosImuSensorPrivate::OnUpdate, impl_.get()));
}

void GazeboRosImuSensorPrivate::OnUpdate() {
  msg_->header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(sensor_->LastUpdateTime());
  msg_->orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(sensor_->Orientation());

  msg_->angular_velocity.x = sensor_->AngularVelocity().X() + angular_velocity_bias_.GenerateNoise();
  msg_->angular_velocity.y = sensor_->AngularVelocity().Y() + angular_velocity_bias_.GenerateNoise();
  msg_->angular_velocity.z = sensor_->AngularVelocity().Z() + angular_velocity_bias_.GenerateNoise();

  msg_->linear_acceleration.x = sensor_->LinearAcceleration().X() + linear_acceleration_bias_.GenerateNoise();
  msg_->linear_acceleration.y = sensor_->LinearAcceleration().Y() + linear_acceleration_bias_.GenerateNoise();
  msg_->linear_acceleration.z = sensor_->LinearAcceleration().Z() + linear_acceleration_bias_.GenerateNoise();

  pub_->publish(*msg_);

  bias_msg_->header.stamp = msg_->header.stamp;
  bias_msg_->bias_acc.x = linear_acceleration_bias_.GenerateNoise();
  bias_msg_->bias_acc.y = linear_acceleration_bias_.GenerateNoise();
  bias_msg_->bias_acc.z = linear_acceleration_bias_.GenerateNoise();
  bias_msg_->bias_gyr.x = angular_velocity_bias_.GenerateNoise();
  bias_msg_->bias_gyr.y = angular_velocity_bias_.GenerateNoise();
  bias_msg_->bias_gyr.z = angular_velocity_bias_.GenerateNoise();

  bias_pub_->publish(*bias_msg_);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosImuSensor)

}  // namespace gazebo_plugins

