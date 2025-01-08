// gazebo_ros_fixposition.hpp

#ifndef GAZEBO_ROS_FIXPOSITION_HPP
#define GAZEBO_ROS_FIXPOSITION_HPP
#include <iostream>
#include <cstring>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <array>
#include <chrono>
#include <sstream>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <std_srvs/srv/trigger.hpp>

// Include custom message headers
#include "fixposition_msgs/msg/odomenu.hpp"
#include "fixposition_msgs/msg/odometry.hpp"
#include "fixposition_msgs/msg/odomstatus.hpp"
#include "fixposition_msgs/msg/odomsh.hpp"
#include "fixposition_msgs/msg/llh.hpp"

// Include the GazeboFixpositionHelper class
#include "gazebo_fixposition_helper.hpp"

namespace gazebo_plugins {

/**
 * @brief Gazebo Model Plugin for fixed positioning using GPS data.
 */
class GazeboFixpositionPlugin : public gazebo::ModelPlugin {
public:
    GazeboFixpositionPlugin();
    /**
     * @brief Load the plugin.
     * @param _model Pointer to the model.
     * @param _sdf SDF element.
     */
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
    // Callback functions
    void GpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void OnUpdate(const gazebo::common::UpdateInfo& info);

    // Transformation functions (delegated to GazeboFixpositionHelper)
    ignition::math::Vector3d ConvertLocalToGeographic(const ignition::math::Pose3d& localPose);

    // Publish functions
    void PublishENUFrame();
    ignition::math::Pose3d TransformToENUFrame(const ignition::math::Pose3d& currentPose, const ignition::math::Pose3d& initialPose);
    void UpdateOdometryWorld();
    void PublishTransforms(const Eigen::Vector3d& ecefPosition, const Eigen::Quaterniond& ecefOrientation, const ignition::math::Pose3d& ENU_pose);
    void PublishIMUAndYPR(const ignition::math::Pose3d& pose);
    void PublishFusionMessages(const rclcpp::Time&  current_time);
    void UpdatePOIPosition();
    void PublishOdometryAndTFAndFusionMessages();
    void PublishFusionMessagesConsistentNoiseDrift(double poiX, double poiY, double poiZ,
                                               double driftX, double driftY, double driftZ);
    void ResetPOISHDrift(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response
    );
    // ROS and Gazebo components
    gazebo_ros::Node::SharedPtr ros_node_;
    gazebo::event::ConnectionPtr update_connection_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr poi_gps_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_poish_service_;

    // ODOMETRY publishers
    rclcpp::Publisher<fixposition_msgs::msg::ODOMENU>::SharedPtr odom_enu_pub_;
    rclcpp::Publisher<fixposition_msgs::msg::ODOMETRY>::SharedPtr odometry_pub_;
    rclcpp::Publisher<fixposition_msgs::msg::ODOMSTATUS>::SharedPtr odom_status_pub_;
    rclcpp::Publisher<fixposition_msgs::msg::ODOMSH>::SharedPtr odom_sh_pub_;

    // Other publishers and broadcasters
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ecef_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ecef_odom_pub_, enu_odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_ypr_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ypr_pub_;
    rclcpp::Publisher<fixposition_msgs::msg::LLH>::SharedPtr llh_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr sh_odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr poi_imu_pub_;

    gazebo::common::SphericalCoordinates sphericalCoords_;

    // Odometry messages
    nav_msgs::msg::Odometry odom_, odom_enu, odom_ecef;

    gazebo::physics::ModelPtr model_;
    ignition::math::Pose3d initialPose;
    bool initialPoseSet = false;

    // Member variables to store the latest GPS data
    double last_gps_latitude = 0.0;
    double last_gps_longitude = 0.0;
    double last_gps_altitude = 0.0;

    gazebo::common::Time last_update_time_;
    gazebo::common::Time update_period_;
    bool got_gps = false;

    // Fusion publishing rate
    gazebo::common::Time publish_period_fusion_;
    gazebo::common::Time last_publish_time_fusion_;

    // POI publishing rate
    gazebo::common::Time publish_period_poi_;
    gazebo::common::Time last_publish_time_poi_;

    // Configurable POI pose
    ignition::math::Pose3d poi_pose_;

    // Instance of GazeboFixpositionHelper
    GazeboFixpositionHelper fixposition_helper_;

    // Smooth odometry drift
    double driftX_;
    double driftY_;
    double driftZ_;

};

} // namespace gazebo_plugins

#endif // GAZEBO_ROS_FIXPOSITION_HPP

