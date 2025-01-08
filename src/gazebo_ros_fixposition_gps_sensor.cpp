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
#include <fixposition_msgs/msg/gnssant.hpp>
#include <fixposition_msgs/msg/gnsscorr.hpp>  
#include "fixposition_sensor_pkg/NoiseGenerator.hpp"
#include <iostream>
#include <memory>
#include <string>

namespace gazebo_plugins {

class GazeboRosGpsSensorPrivate {
public:
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<fixposition_msgs::msg::GNSSANT>::SharedPtr gnss_ant_pub_;
  rclcpp::Publisher<fixposition_msgs::msg::GNSSCORR>::SharedPtr gnss_corr_pub_;
  
  sensor_msgs::msg::NavSatFix::SharedPtr msg_;
  geometry_msgs::msg::Vector3Stamped::SharedPtr msg_vel_;
  fixposition_msgs::msg::GNSSANT::SharedPtr msg_gnss_ant_;
  fixposition_msgs::msg::GNSSCORR::SharedPtr msg_gnss_corr_; 
  
  gazebo::sensors::GpsSensorPtr sensor_;
  gazebo::event::ConnectionPtr sensor_update_event_;
  
  NoiseGenerator noise_generator_;  

  void OnUpdate();
};

GazeboRosGpsSensor::GazeboRosGpsSensor() : impl_(std::make_unique<GazeboRosGpsSensorPrivate>()) {}

GazeboRosGpsSensor::~GazeboRosGpsSensor() {}

void GazeboRosGpsSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

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
  impl_->gnss_ant_pub_ = impl_->ros_node_->create_publisher<fixposition_msgs::msg::GNSSANT>(
    "/fixposition/fpa/gnssant", qos.get_publisher_qos("/fixposition/fpa/gnssant", rclcpp::SensorDataQoS().reliable()));
  impl_->gnss_corr_pub_ = impl_->ros_node_->create_publisher<fixposition_msgs::msg::GNSSCORR>(
    "/fixposition/fpa/gnsscorr", qos.get_publisher_qos("/fixposition/fpa/gnsscorr", rclcpp::SensorDataQoS().reliable()));

  auto msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
  auto msg_vel = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
  auto msg_gnss_ant = std::make_shared<fixposition_msgs::msg::GNSSANT>();
  auto msg_gnss_corr = std::make_shared<fixposition_msgs::msg::GNSSCORR>();

  msg->header.frame_id = msg_vel->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Set noise parameters from SDF
  std::string noise_type_str = "gaussian";
  if (_sdf->HasElement("noise_type")) {
    noise_type_str = _sdf->Get<std::string>("noise_type");
  }

  // Determine noise type
  if (noise_type_str == "gaussian") {
    impl_->noise_generator_.SetGaussianParameters(0.0, 1.0);  // Default values, will be overridden if specified
  } else if (noise_type_str == "uniform") {
    impl_->noise_generator_.SetUniformParameters(-1.0, 1.0);
  } else if (noise_type_str == "bias_drift") {
    impl_->noise_generator_.SetBiasDriftParameters(0.0, 0.01);
  } else if (noise_type_str == "white_noise") {
    impl_->noise_generator_.SetWhiteNoiseParameters(0.0, 1.0);
  } else if (noise_type_str == "exponential") {
    impl_->noise_generator_.SetExponentialParameters(1.0);
  } else if (noise_type_str == "random_walk") {
    impl_->noise_generator_.SetRandomWalkParameters(0.1);
  } else {
    RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), "Unknown noise type, defaulting to Gaussian.");
    impl_->noise_generator_.SetGaussianParameters(0.0, 1.0);
  }

  // Gaussian noise parameters
  double gaussian_mean = _sdf->HasElement("gaussian_mean") ? _sdf->Get<double>("gaussian_mean") : 0.0;
  double gaussian_stddev = _sdf->HasElement("gaussian_stddev") ? _sdf->Get<double>("gaussian_stddev") : 1.0;
  impl_->noise_generator_.SetGaussianParameters(gaussian_mean, gaussian_stddev);

  // Uniform noise parameters
  double uniform_min = _sdf->HasElement("uniform_min") ? _sdf->Get<double>("uniform_min") : -1.0;
  double uniform_max = _sdf->HasElement("uniform_max") ? _sdf->Get<double>("uniform_max") : 1.0;
  impl_->noise_generator_.SetUniformParameters(uniform_min, uniform_max);

  // Bias Drift noise parameters
  double initial_bias = _sdf->HasElement("initial_bias") ? _sdf->Get<double>("initial_bias") : 0.0;
  double drift_rate = _sdf->HasElement("drift_rate") ? _sdf->Get<double>("drift_rate") : 0.01;
  impl_->noise_generator_.SetBiasDriftParameters(initial_bias, drift_rate);

  // White Noise parameters
  double white_noise_mean = _sdf->HasElement("white_noise_mean") ? _sdf->Get<double>("white_noise_mean") : 0.0;
  double white_noise_stddev = _sdf->HasElement("white_noise_stddev") ? _sdf->Get<double>("white_noise_stddev") : 1.0;
  impl_->noise_generator_.SetWhiteNoiseParameters(white_noise_mean, white_noise_stddev);

  // Exponential noise parameters
  double exponential_lambda = _sdf->HasElement("exponential_lambda") ? _sdf->Get<double>("exponential_lambda") : 1.0;
  impl_->noise_generator_.SetExponentialParameters(exponential_lambda);

  // Random Walk noise parameters
  double random_walk_step = _sdf->HasElement("random_walk_step") ? _sdf->Get<double>("random_walk_step") : 0.1;
  impl_->noise_generator_.SetRandomWalkParameters(random_walk_step);

  // Set message pointers
  impl_->msg_ = msg;
  impl_->msg_vel_ = msg_vel;
  impl_->msg_gnss_ant_ = msg_gnss_ant;
  impl_->msg_gnss_corr_ = msg_gnss_corr;

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&GazeboRosGpsSensorPrivate::OnUpdate, impl_.get()));
}

void GazeboRosGpsSensorPrivate::OnUpdate() {
  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosGpsSensorPrivate::OnUpdate");
  IGN_PROFILE_BEGIN("fill ROS message");
  #endif

  // Add noise to the GPS measurement
  double noisy_latitude = sensor_->Latitude().Degree() ;
  double noisy_longitude = sensor_->Longitude().Degree() ;
  double noisy_altitude = sensor_->Altitude() ;

  msg_->header.stamp = msg_vel_->header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(sensor_->LastUpdateTime());
  msg_->latitude = noisy_latitude;
  msg_->longitude = noisy_longitude;
  msg_->altitude = noisy_altitude;

  // Update covariance based on noise standard deviation
  double covariance_value = std::pow(noise_generator_.GetStandardDeviation(), 2);
  msg_->position_covariance[0] = covariance_value;
  msg_->position_covariance[4] = covariance_value;
  msg_->position_covariance[8] = covariance_value;
  msg_->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  msg_vel_->vector.x = sensor_->VelocityEast();
  msg_vel_->vector.y = sensor_->VelocityNorth();
  msg_vel_->vector.z = sensor_->VelocityUp();

  // Fill GNSSANT message
  msg_gnss_ant_->header.stamp = msg_->header.stamp;
  msg_gnss_ant_->gnss1_state = "active";
  msg_gnss_ant_->gnss1_power = "on";
  msg_gnss_ant_->gnss1_age = 10;
  msg_gnss_ant_->gnss2_state = "inactive";
  msg_gnss_ant_->gnss2_power = "off";
  msg_gnss_ant_->gnss2_age = 20;

  // Fill GNSSCORR message
  msg_gnss_corr_->header.stamp = msg_->header.stamp;
  msg_gnss_corr_->gnss1_fix = 1;
  msg_gnss_corr_->gnss1_nsig_l1 = 5;
  msg_gnss_corr_->gnss1_nsig_l2 = 3;
  msg_gnss_corr_->gnss2_fix = 0;
  msg_gnss_corr_->gnss2_nsig_l1 = 2;
  msg_gnss_corr_->gnss2_nsig_l2 = 1;
  msg_gnss_corr_->corr_latency = 0.5;
  msg_gnss_corr_->corr_update_rate = 1.0;
  msg_gnss_corr_->corr_data_rate = 0.25;
  msg_gnss_corr_->corr_msg_rate = 0.05;
  msg_gnss_corr_->sta_id = 1234;
  msg_gnss_corr_->sta_llh.x = 36.71722754548088;
  msg_gnss_corr_->sta_llh.y = -4.489284968712882;
  msg_gnss_corr_->sta_llh.z = 2.8740682220086455;
  msg_gnss_corr_->sta_dist = 5000;

  pub_->publish(*msg_);
  vel_pub_->publish(*msg_vel_);
  gnss_ant_pub_->publish(*msg_gnss_ant_);
  gnss_corr_pub_->publish(*msg_gnss_corr_);

  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  #endif
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosGpsSensor)

}  // namespace gazebo_plugins

