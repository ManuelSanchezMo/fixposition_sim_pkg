#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo_ros/node.hpp>
#include "gazebo/common/common.hh"
#include <iostream>
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
#include <nav_msgs/msg/odometry.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <ignition/math.hh>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <chrono>

// WGS-84 Earth model constants
const double a = 6378137.0;  // Semi-major axis
const double e_squared = 0.00669437999014;  // Eccentricity squared

// Function to convert latitude and longitude in degrees to ECEF coordinates
geometry_msgs::msg::Point ConvertToECEF(double latitude, double longitude, double altitude) {
    double lat_rad = latitude * M_PI / 180.0;  // Convert latitude to radians
    double lon_rad = longitude * M_PI / 180.0; // Convert longitude to radians

    // Calculate ECEF coordinates
    geometry_msgs::msg::Point ecef;
    double N = a / sqrt(1 - e_squared * sin(lat_rad) * sin(lat_rad));
    ecef.x = (N + altitude) * cos(lat_rad) * cos(lon_rad);
    ecef.y = (N + altitude) * cos(lat_rad) * sin(lon_rad);
    ecef.z = (N * (1 - e_squared) + altitude) * sin(lat_rad);

    return ecef;
}

namespace gazebo_plugins {
    class GazeboFixpositionPlugin : public gazebo::ModelPlugin {
    public:
        GazeboFixpositionPlugin() : gazebo::ModelPlugin() {
            std::cout << "Minimal Working Plugin constructed!\n";
        }

        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
            std::cout << "Minimal Working Plugin loaded!\n";

            // Initialize model and ROS node
            model_ = _model;
            ros_node_ = gazebo_ros::Node::Get(_sdf);

            // Initialize publishers and TF broadcasters
            const gazebo_ros::QoS & qos = ros_node_->get_qos();
            ecef_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "~/ecef", qos.get_publisher_qos("~/ecef", rclcpp::SensorDataQoS().reliable()));
            enu_odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(
                "fixposition/odometry_enu", qos.get_publisher_qos("fixposition/odometry_enu", rclcpp::SensorDataQoS().reliable()));
            ecef_odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(
                "fixposition/odometry", qos.get_publisher_qos("fixposition/odometry", rclcpp::SensorDataQoS().reliable()));

            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);
            static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(ros_node_);
            // Create a subscriber to the GPS topic
            gps_sub_ = ros_node_->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/fixposition/navsatfix", 10, 
                std::bind(&GazeboFixpositionPlugin::GpsCallback, this, std::placeholders::_1));
            imu_ypr_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Vector3>("/fixposition/imu_ypr", 10);
            // Initialize the /fixposition/ypr publisher
            ypr_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Vector3>("/fixposition/ypr", 10);

            // Connect to the update event
            update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                std::bind(&GazeboFixpositionPlugin::OnUpdate, this, std::placeholders::_1));
            // Default update frequency in Hz
            double default_update_frequency = 10.0;  // Example: 10 Hz

            // Check if the update_frequency parameter exists and use it, otherwise use the default
            if (_sdf->HasElement("update_frequency")) {
                default_update_frequency = _sdf->Get<double>("update_frequency");
            }

            // Convert frequency to update period (in seconds)
            update_period_ = 1.0 / default_update_frequency;
            // Define and broadcast the first transform (FP_VRTK to FP_CAM)
            geometry_msgs::msg::TransformStamped tf_vrtk_cam;
            tf_vrtk_cam.header.frame_id = "FP_VRTK";
            tf_vrtk_cam.child_frame_id = "FP_CAM";
            tf_vrtk_cam.transform.translation.x = 0.03383;
            tf_vrtk_cam.transform.translation.y = 0.00373;
            tf_vrtk_cam.transform.translation.z = -0.01227;
            tf_vrtk_cam.transform.rotation.x = 0.502196;
            tf_vrtk_cam.transform.rotation.y = 0.50099;
            tf_vrtk_cam.transform.rotation.z = 0.498972;
            tf_vrtk_cam.transform.rotation.w = 0.49783;
            static_tf_broadcaster_->sendTransform(tf_vrtk_cam);

            // Define and broadcast the second transform (FP_POI to FP_VRTK)
            geometry_msgs::msg::TransformStamped tf_poi_vrtk;
            tf_poi_vrtk.header.frame_id = "FP_POI";
            tf_poi_vrtk.child_frame_id = "FP_VRTK";
            tf_poi_vrtk.transform.translation.x = -0.0;
            tf_poi_vrtk.transform.translation.y = -0.0;
            tf_poi_vrtk.transform.translation.z = -0.0;
            tf_poi_vrtk.transform.rotation.x = 0.0;
            tf_poi_vrtk.transform.rotation.y = 0.0;
            tf_poi_vrtk.transform.rotation.z = 0.0;
            tf_poi_vrtk.transform.rotation.w = 1.0;
            static_tf_broadcaster_->sendTransform(tf_poi_vrtk);
            }

    // Add a callback function for the GPS topic
    void GpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        // Process the incoming GPS data
        // For example, store the latest GPS data in member variables
        last_gps_latitude = msg->latitude;
        last_gps_longitude = msg->longitude;
        last_gps_altitude = msg->altitude;
        got_gps = true;
    }

    private:
        void OnUpdate(const gazebo::common::UpdateInfo& info) {
            double current_time_seconds = info.simTime.Double();
             double last_update_time_seconds = last_update_time_.Double();
            double elapsed_time = current_time_seconds - last_update_time_seconds;
            std::cout << "Updating Minimal Working Plugin!\n";
        if (elapsed_time >= update_period_) {
                if (!initialPoseSet) {
                    initialPose = model_->WorldPose();
                    PublishENUFrame();
                }

                // Update odometry world based on current model pose
                UpdateOdometryWorld();
                last_update_time_ = info.simTime;
            }
        }

        ignition::math::Vector3d ConvertLocalToGeographic(const ignition::math::Pose3d& localPose) {
            // Convert local pose to geographic coordinates
            return sphericalCoords.SphericalFromLocal(localPose.Pos());
        }

        Eigen::Matrix3d RotEnuEcef(double lat, double lon) {
            double sLat = sin(lat);
            double cLat = cos(lat);
            double sLon = sin(lon);
            double cLon = cos(lon);

            Eigen::Matrix3d R;
            R << -sLon, -sLat * cLon, cLat * cLon,
                  cLon, -sLat * sLon, cLat * sLon,
                  0,     cLat,        sLat;

            return R;
        }
        geometry_msgs::msg::Quaternion ConvertMatrixToQuaternion(const Eigen::Matrix3d& matrix) {
            Eigen::Quaterniond quat(matrix);
            geometry_msgs::msg::Quaternion quaternion_msg;
            quaternion_msg.x = quat.x();
            quaternion_msg.y = quat.y();
            quaternion_msg.z = quat.z();
            quaternion_msg.w = quat.w();
            return quaternion_msg;
        }
        constexpr double deg2rad(double degrees) {
            return degrees * M_PI / 180.0;
        }

        void PublishENUFrame() {
            if (got_gps) {
                geometry_msgs::msg::TransformStamped enuTransform;
                enuTransform.header.stamp = ros_node_->now();
                enuTransform.header.frame_id = "ECEF";
                enuTransform.child_frame_id = "FP_ENU0";
                auto ecefPose = ConvertToECEF(last_gps_latitude, last_gps_longitude, last_gps_altitude);

                enuTransform.transform.translation.x = ecefPose.x;
                enuTransform.transform.translation.y = ecefPose.y;
                enuTransform.transform.translation.z = ecefPose.z;

                // Calculate the ENU to ECEF rotation matrix
                Eigen::Matrix3d enuToEcefRotation = RotEnuEcef(deg2rad(last_gps_latitude), deg2rad(last_gps_longitude));

                // Convert the rotation matrix to a quaternion and set it to the transform
                enuTransform.transform.rotation = ConvertMatrixToQuaternion(enuToEcefRotation);

                static_tf_broadcaster_->sendTransform(enuTransform);
                initialPoseSet = true;
            }

        }

        ignition::math::Pose3d TransformToENUFrame(const ignition::math::Pose3d& currentPose, const ignition::math::Pose3d& initialPose) {
            // Create a new pose for the transformed position
            ignition::math::Pose3d transformedPose = currentPose;
            // Subtract initial position from current position to translate
            transformedPose.Pos() = currentPose.Pos() - initialPose.Pos();

            return transformedPose;
        }

        void UpdateOdometryWorld() {
            // Get robot's pose and velocity in the Gazebo world frame
            auto pose = model_->WorldPose();
            auto linear = model_->WorldLinearVel();
            auto angular = model_->WorldAngularVel();

            // Use the latest GPS data for ECEF conversion
            auto ecefPose = ConvertToECEF(last_gps_latitude, last_gps_longitude, last_gps_altitude);

            // Get the rotation matrix from ENU to ECEF
            Eigen::Matrix3d enuToEcefRotation = RotEnuEcef(last_gps_latitude * M_PI / 180.0, last_gps_longitude * M_PI / 180.0);

            // Get the robot's orientation in Gazebo's frame as a rotation matrix
            Eigen::Quaterniond gazeboOrientation(pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z());
            Eigen::Matrix3d gazeboRotationMatrix = gazeboOrientation.toRotationMatrix();

            // Apply the transformation
            Eigen::Matrix3d ecefRotationMatrix = enuToEcefRotation * gazeboRotationMatrix;

            // Convert back to a quaternion
            Eigen::Quaterniond ecefOrientation(ecefRotationMatrix);

            // Set the orientation in odom_ecef message
            odom_ecef.pose.pose.position.x = ecefPose.x + noise_dist_x_(generator_);
            odom_ecef.pose.pose.position.y = ecefPose.y + noise_dist_y_(generator_);
            odom_ecef.pose.pose.position.z = ecefPose.z + noise_dist_z_(generator_);

            odom_ecef.pose.pose.orientation.x = ecefOrientation.x();
            odom_ecef.pose.pose.orientation.y = ecefOrientation.y();
            odom_ecef.pose.pose.orientation.z = ecefOrientation.z();
            odom_ecef.pose.pose.orientation.w = ecefOrientation.w();
            odom_ecef.twist.twist.linear.x = model_->RelativeLinearVel().X();;
            odom_ecef.twist.twist.angular.z = model_->RelativeAngularVel().Z();;
            odom_ecef.child_frame_id = "FP_POI";
            odom_ecef.header.frame_id = "ECEF";
            // Get the ENU pose
            auto ENU_pose = TransformToENUFrame(pose, initialPose);
            odom_enu.child_frame_id = "FP_POI";
            odom_enu.header.frame_id = "FP_ENU0";
            odom_enu.pose.pose.position.x = ENU_pose.Pos().X();
            odom_enu.pose.pose.position.y = ENU_pose.Pos().Y();
            odom_enu.pose.pose.position.z = ENU_pose.Pos().Z();

            ignition::math::Quaterniond orientation = pose.Rot(); // Using global pose orientation, as X axis is aligned with EAST.
            odom_enu.pose.pose.orientation.x = orientation.X();
            odom_enu.pose.pose.orientation.y = orientation.Y();
            odom_enu.pose.pose.orientation.z = orientation.Z();
            odom_enu.pose.pose.orientation.w = orientation.W();
            odom_enu.twist.twist.linear.x = model_->RelativeLinearVel().X();;
            odom_enu.twist.twist.angular.z = model_->RelativeAngularVel().Z();;
            // Set the diagonal elements (variances)
            odom_ecef.pose.covariance[0] = 0.00034;   // Variance for x
            odom_ecef.pose.covariance[7] = 0.00047;   // Variance for y
            odom_ecef.pose.covariance[14] = 0.00025;  // Variance for z
            // Assuming variance for roll, pitch, yaw at indices 21, 28, 35 (if available)
            odom_ecef.pose.covariance[21] =  1.0e-05;      // Variance for roll
            odom_ecef.pose.covariance[28] = 0.0;      // Variance for pitch
            odom_ecef.pose.covariance[35] = 1.0e-05;  // Variance for yaw

            // Set the off-diagonal elements (covariances)
            odom_ecef.pose.covariance[1] = 2.0e-05;   // Covariance between x and y
            odom_ecef.pose.covariance[2] = 5.0e-05;   // Covariance between x and z
            odom_ecef.pose.covariance[6] = 2.0e-05;   // Covariance between y and x
            odom_ecef.pose.covariance[8] = 1.0e-05;   // Covariance between y and z
            odom_ecef.pose.covariance[12] = 5.0e-05;  // Covariance between z and x
            odom_ecef.pose.covariance[13] = 1.0e-05;  // Covariance between z and y

            geometry_msgs::msg::TransformStamped transformStamped;

            // Set the header information
            transformStamped.header.stamp = ros_node_->now(); // Use the current time
            transformStamped.header.frame_id = "ECEF"; // Replace with your world frame
            transformStamped.child_frame_id = "FP_POI"; // Replace with your robot's frame

            // Set the position
            transformStamped.transform.translation.x = ecefPose.x;
            transformStamped.transform.translation.y = ecefPose.y;
            transformStamped.transform.translation.z = ecefPose.z;

            // Set the orientation
            transformStamped.transform.rotation.x = ecefOrientation.x();
            transformStamped.transform.rotation.y = ecefOrientation.y();
            transformStamped.transform.rotation.z = ecefOrientation.z();
            transformStamped.transform.rotation.w = ecefOrientation.w();
            tf_broadcaster_->sendTransform(transformStamped);


            // Set the header information
            transformStamped.header.stamp = ros_node_->now(); // Use the current time
            transformStamped.header.frame_id = "FP_ENU0"; // Replace with your world frame
            transformStamped.child_frame_id = "FP_POI"; // Replace with your robot's frame

            // Set the position
            transformStamped.transform.translation.x = ENU_pose.Pos().X();
            transformStamped.transform.translation.y = ENU_pose.Pos().Y();
            transformStamped.transform.translation.z = ENU_pose.Pos().Z();

            // Set the orientation
            transformStamped.transform.rotation.x = orientation.X();
            transformStamped.transform.rotation.y = orientation.Y();
            transformStamped.transform.rotation.z = orientation.Z();
            transformStamped.transform.rotation.w = orientation.W();
            //tf_broadcaster_->sendTransform(transformStamped);
            // Calculate FP_IMUH orientation: retain yaw, zero out pitch and roll
            ignition::math::Quaterniond fp_imuh_orientation(pose.Rot().Pitch(), pose.Rot().Roll(), pose.Rot().Yaw());

            // Construct and send the dynamic transform from FP_POI to FP_IMUH
            geometry_msgs::msg::TransformStamped tf_poi_imuh;
            tf_poi_imuh.header.stamp = ros_node_->now();
            tf_poi_imuh.header.frame_id = "FP_POI";
            tf_poi_imuh.child_frame_id = "FP_IMUH";
            tf_poi_imuh.transform.translation.x = 0.0;  // Assuming same origin as FP_POI
            tf_poi_imuh.transform.translation.y = 0.0;
            tf_poi_imuh.transform.translation.z = 0.0;
            tf_poi_imuh.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(fp_imuh_orientation);

            tf_broadcaster_->sendTransform(tf_poi_imuh);

            // Publish ECEF and ENU odometry
            ecef_odom_pub_->publish(odom_ecef);
            enu_odom_pub_->publish(odom_enu);
            geometry_msgs::msg::Vector3 imu_ypr_msg;
            auto roll_pitch_yaw = pose.Rot().Euler();

            imu_ypr_msg.x = 0.0;           // Fixed value as per your description
            imu_ypr_msg.y = roll_pitch_yaw.Y();// Pitch
            imu_ypr_msg.z = roll_pitch_yaw.X(); // Roll

            imu_ypr_pub_->publish(imu_ypr_msg);

            geometry_msgs::msg::Vector3 ypr_msg;
            auto roll_pitch_yaw_ypr = pose.Rot().Euler();

            ypr_msg.x = roll_pitch_yaw_ypr.Z(); // Yaw
            ypr_msg.y = roll_pitch_yaw_ypr.Y(); // Pitch
            ypr_msg.z = roll_pitch_yaw_ypr.X(); // Roll

            ypr_pub_->publish(ypr_msg);
        }

        // Member variables
        gazebo_ros::Node::SharedPtr ros_node_;
        gazebo::event::ConnectionPtr update_connection_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ecef_pub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
        std::shared_ptr<geometry_msgs::msg::TransformStamped> enu_tf, ecef_tf;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ecef_odom_pub_, enu_odom_pub_;
        nav_msgs::msg::Odometry odom_, odom_enu, odom_ecef;
        gazebo::physics::ModelPtr model_;
        gazebo::common::SphericalCoordinates sphericalCoords;
        ignition::math::Pose3d initialPose;
        bool initialPoseSet = false;
        // Add a ROS subscriber for the GPS data
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

        // Member variables to store the latest GPS data
        double last_gps_latitude = 0.0;
        double last_gps_longitude = 0.0;
        double last_gps_altitude = 0.0;

        gazebo::common::Time last_update_time_;
        double update_period_;
        double std_dev_x_ = sqrt(0.00048); 
        double std_dev_y_ = sqrt(0.00036);
        double std_dev_z_ = sqrt(0.00033);

        std::default_random_engine generator_;
        std::normal_distribution<double> noise_dist_x_{0.0, std_dev_x_};
        std::normal_distribution<double> noise_dist_y_{0.0, std_dev_y_};
        std::normal_distribution<double> noise_dist_z_{0.0, std_dev_z_};
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_ypr_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ypr_pub_;
        bool got_gps = false;
    };

    GZ_REGISTER_MODEL_PLUGIN(GazeboFixpositionPlugin)
} // namespace gazebo_plugins