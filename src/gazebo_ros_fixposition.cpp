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
#include <fixposition_sensor_pkg/gazebo_ros_fixposition_gps_sensor.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
// Constants for WGS-84
const double a = 6378137.0; // Semi-major axis
const double e_squared = 0.00669437999014; // Eccentricity squared
/*Eigen::Matrix3d RotEnuEcef(double lat, double lon) {
    const double s_lon = sin(lon);
    const double c_lon = cos(lon);
    const double s_lat = sin(lat);
    const double c_lat = cos(lat);
    const double m00 = -s_lon;
    const double m01 = c_lon;
    const double m02 = 0;
    const double m10 = -c_lon * s_lat;
    const double m11 = -s_lon * s_lat;
    const double m12 = c_lat;
    const double m20 = c_lon * c_lat;
    const double m21 = s_lon * c_lat;
    const double m22 = s_lat;

    Eigen::Matrix3d res;
    // This initializes the matrix row by row
    res << m00, m01, m02, m10, m11, m12, m20, m21, m22;

    return res;
}*/
double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}
geometry_msgs::msg::Point ConvertToECEF(const ignition::math::Pose3d& pose) {
    // Assume pose.Pos() gives the latitude, longitude, and altitude
    // You'll need to adapt this part to get actual GPS data from your simulation
    double latitude = pose.Pos().X();
    double longitude = pose.Pos().Y();
    double altitude = pose.Pos().Z();

    // Convert latitude and longitude to radians
    double lat_rad = deg2rad(latitude);
    double lon_rad = deg2rad(longitude);

    // Calculate ECEF coordinates
    geometry_msgs::msg::Point ecef;
    double N = a / sqrt(1 - e_squared * pow(sin(lat_rad), 2));
    ecef.x = (N + altitude) * cos(lat_rad) * cos(lon_rad);
    ecef.y = (N + altitude) * cos(lat_rad) * sin(lon_rad);
    ecef.z = ((1 - e_squared) * N + altitude) * sin(lat_rad);

    return ecef;
}
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
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ecef_pub_;
  /// Publish latest gps data to ROS
  /// Transform Broadcaster to publish pose as tf
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // Initial pose in ECEF coordinates
  geometry_msgs::msg::Point initial_pose_ecef;
  // Flag to check if initial pose is set
  bool initial_pose_set = false;
// Static Transform Broadcaster
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  // Method to capture and store initial pose
  // Odometry publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ecef_odom_pub_, enu_odom_pub_;
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
 impl_->ecef_pub_ = impl_->ros_node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "~/ecef", qos.get_publisher_qos("~/ecef", rclcpp::SensorDataQoS().reliable()));
  impl_->enu_odom_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
        "fixposition/odometry_enu", qos.get_publisher_qos("fixposition/odometry_enu", rclcpp::SensorDataQoS().reliable()));
  impl_->ecef_odom_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
        "fixposition/odometry", qos.get_publisher_qos("fixposition/odometry", rclcpp::SensorDataQoS().reliable()));
  // Create message to be reused
   // Initialize TF broadcaster
    impl_->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);
  auto msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
  auto msg_vel = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
// Initialize Static TF broadcaster
    impl_->static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(impl_->ros_node_);
  // Get frame for message
  msg->header.frame_id = msg_vel->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);
  //impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
  //"/odometry", qos.get_publisher_qos("/odometry", rclcpp::SensorDataQoS().reliable()));
  // Fill covariances
  using SNT = gazebo::sensors::SensorNoiseType;
  msg->position_covariance[0] = 0.001;
  msg->position_covariance[4] = 0.001;
  msg->position_covariance[8] = 0.001;
  msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  // Fill gps status
  msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  msg->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  impl_->msg_ = msg;
  impl_->msg_vel_ = msg_vel;

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
  std::bind(&GazeboRosGpsSensorPrivate::OnUpdate, impl_.get()));
        std::cout <<"eeeeeeeeeeeeeeeeeeeeee"<< std::endl;

      std::cout <<"eeeeeeeeeeeeeeeeeeeeee"<< std::endl;

      std::cout <<"eeeeeeeeeeeeeeeeeeeeee"<< std::endl;
*(int*)0 = 0;
}

void GazeboRosGpsSensorPrivate::OnUpdate()
{
  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosGpsSensorPrivate::OnUpdate");
  IGN_PROFILE_BEGIN("fill ROS message");
  #endif
        std::cout << "update fixpositon " <<std::endl;

  // Fill messages with the latest sensor data
  msg_->header.stamp = msg_vel_->header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    sensor_->LastUpdateTime());
  msg_->latitude = sensor_->Latitude().Degree();
  msg_->longitude = sensor_->Longitude().Degree();
  msg_->altitude = sensor_->Altitude();
  msg_vel_->vector.x = sensor_->VelocityEast();
  msg_vel_->vector.y = sensor_->VelocityNorth();
  msg_vel_->vector.z = sensor_->VelocityUp();
  auto pose = sensor_->Pose();
  auto velocity = sensor_->VelocityENU();
// GPS to ECEF conversion
    double latitude_rad = deg2rad(msg_->latitude);
    double longitude_rad = deg2rad(msg_->longitude);
    double altitude = msg_->altitude;
        std::cout << "no here " <<std::endl;

    double N = a / sqrt(1 - e_squared * pow(sin(latitude_rad), 2));

    double X = (N + altitude) * cos(latitude_rad) * cos(longitude_rad);
    double Y = (N + altitude) * cos(latitude_rad) * sin(longitude_rad);
    double Z = ((1 - e_squared) * N + altitude) * sin(latitude_rad);
  if (!initial_pose_set){
      geometry_msgs::msg::TransformStamped StatictransformStamped;
    //Eigen::Matrix3d rot_matrix = RotEnuEcef( latitude_rad,  longitude_rad);
    //Eigen::Quaterniond quaternion(rot_matrix);

    StatictransformStamped.header.stamp = msg_->header.stamp;
    StatictransformStamped.header.frame_id = "ecef";
    StatictransformStamped.child_frame_id = "FP_ENU"; // or your robot's frame id
    StatictransformStamped.transform.translation.x = X;
    StatictransformStamped.transform.translation.y = Y;
    StatictransformStamped.transform.translation.z = Z;
    StatictransformStamped.transform.rotation.x = 0.0;
    StatictransformStamped.transform.rotation.y = 0.0;
    StatictransformStamped.transform.rotation.z = 0.0;
    StatictransformStamped.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(StatictransformStamped);
    initial_pose_set = true;
  }
    // Create and publish ECEF pose message
    geometry_msgs::msg::PoseWithCovarianceStamped ecef_msg;
    ecef_msg.header.stamp = msg_->header.stamp;
    ecef_msg.header.frame_id = "ecef";
    ecef_msg.pose.pose.position.x = X;
    ecef_msg.pose.pose.position.y = Y;
    ecef_msg.pose.pose.position.z = Z;
    ecef_msg.pose.pose.orientation.w = 1.0; // Assuming no orientation information

    ecef_pub_->publish(ecef_msg);

        // Broadcast the transform (ECEF)
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = msg_->header.stamp;
    transformStamped.header.frame_id = "ecef";
    transformStamped.child_frame_id = "base_link"; // or your robot's frame id
    transformStamped.transform.translation.x = X;
    transformStamped.transform.translation.y = Y;
    transformStamped.transform.translation.z = Z;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;


        // Create and publish Odometry message
    nav_msgs::msg::Odometry odometry_msg;
    odometry_msg.header.stamp = msg_->header.stamp;
    odometry_msg.header.frame_id = "ecef";
    odometry_msg.child_frame_id = "fp_poi"; // FP_POI frame ID
    odometry_msg.pose.pose.position.x = X; // ECEF X
    odometry_msg.pose.pose.position.y = Y; // ECEF Y
    odometry_msg.pose.pose.position.z = Z; // ECEF Z
    // Set orientation and velocity as per your requirement
    // ...

    //odometry_pub_->publish(odometry_msg);
  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("publish");
  #endif
  // Publish message
  pub_->publish(*msg_);
  vel_pub_->publish(*msg_vel_);
      std::cout <<"eeeeeeeeeeeeeeeeeeeeee"<< std::endl;
    tf_broadcaster_->sendTransform(transformStamped);

  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  #endif
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosGpsSensor)

}  // namespace gazebo_plugins
