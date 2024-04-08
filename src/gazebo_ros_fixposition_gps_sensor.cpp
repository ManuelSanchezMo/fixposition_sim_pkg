// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <builtin_interfaces/msg/time.hpp>
#include <gazebo_plugins/gazebo_ros_gps_sensor.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <iostream>
#include <memory>
#include <string>

namespace gazebo_plugins
{

class GazeboRosGpsSensorPrivate
{
public:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  /// Publish for gps message
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
  /// Publish for velocity message
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vel_pub_;
  /// GPS message modified each update
  sensor_msgs::msg::NavSatFix::SharedPtr msg_;
  /// Velocity message modified each update
  geometry_msgs::msg::Vector3Stamped::SharedPtr msg_vel_;
  /// GPS sensor this plugin is attached to
  gazebo::sensors::GpsSensorPtr sensor_;
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;

  /// Publish latest gps data to ROS
  void OnUpdate();
};

GazeboRosGpsSensor::GazeboRosGpsSensor()
: impl_(std::make_unique<GazeboRosGpsSensorPrivate>())
{
}

GazeboRosGpsSensor::~GazeboRosGpsSensor()
{
}

void GazeboRosGpsSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::GpsSensor>(_sensor);
  if (!impl_->sensor_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Parent is not a GPS sensor. Exiting.");
    return;
  }

  impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::NavSatFix>(
    "~/out", qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().reliable()));
  impl_->vel_pub_ = impl_->ros_node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "~/vel", qos.get_publisher_qos("~/vel", rclcpp::SensorDataQoS().reliable()));

  // Create message to be reused
  auto msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
  auto msg_vel = std::make_shared<geometry_msgs::msg::Vector3Stamped>();

  // Get frame for message
  msg->header.frame_id = msg_vel->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Fill covariances
  using SNT = gazebo::sensors::SensorNoiseType;
  msg->position_covariance[0] = 0.001;
  msg->position_covariance[4] = 0.001;
  msg->position_covariance[8] = 0.002;
  msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  // Fill gps status
  msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  msg->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  impl_->msg_ = msg;
  impl_->msg_vel_ = msg_vel;

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&GazeboRosGpsSensorPrivate::OnUpdate, impl_.get()));
}

void GazeboRosGpsSensorPrivate::OnUpdate()
{
  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosGpsSensorPrivate::OnUpdate");
  IGN_PROFILE_BEGIN("fill ROS message");
  #endif
  std::cout<<"Fixposition GPS"<<std::endl;
  // Fill messages with the latest sensor data
  msg_->header.stamp = msg_vel_->header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    sensor_->LastUpdateTime());
  msg_->latitude = sensor_->Latitude().Degree();
  msg_->longitude = sensor_->Longitude().Degree();
  msg_->altitude = sensor_->Altitude();
  msg_vel_->vector.x = sensor_->VelocityEast();
  msg_vel_->vector.y = sensor_->VelocityNorth();
  msg_vel_->vector.z = sensor_->VelocityUp();

  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("publish");
  #endif
  // Publish message
  pub_->publish(*msg_);
  vel_pub_->publish(*msg_vel_);
  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  #endif
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosGpsSensor)

}  // namespace gazebo_plugins
