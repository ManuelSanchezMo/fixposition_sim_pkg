#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo_ros/node.hpp>
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
#include <sstream>

// WGS-84 Earth model constants
const double a = 6378137.0;  // Semi-major axis
const double e_squared = 0.00669437999014;  // Eccentricity squared

// Function to convert degrees to radians
constexpr double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

// Function to convert latitude and longitude in degrees to ECEF coordinates
geometry_msgs::msg::Point ConvertToECEF(double latitude, double longitude, double altitude) {
    double lat_rad = deg2rad(latitude);
    double lon_rad = deg2rad(longitude);

    geometry_msgs::msg::Point ecef;
    double N = a / sqrt(1 - e_squared * sin(lat_rad) * sin(lat_rad));
    ecef.x = (N + altitude) * cos(lat_rad) * cos(lon_rad);
    ecef.y = (N + altitude) * cos(lat_rad) * sin(lon_rad);
    ecef.z = (N * (1 - e_squared) + altitude) * sin(lat_rad);

    return ecef;
}

void ECEFToLLA(double x, double y, double z, double& latitude_deg, double& longitude_deg, double& altitude) {
    double b = sqrt(a * a * (1 - e_squared));
    double ep = sqrt((a * a - b * b) / (b * b));
    double p = sqrt(x * x + y * y);
    double th = atan2(a * z, b * p);

    double lon_rad = atan2(y, x);
    double lat_rad = atan2(z + ep * ep * b * pow(sin(th), 3), p - e_squared * a * pow(cos(th), 3));
    double N = a / sqrt(1 - e_squared * sin(lat_rad) * sin(lat_rad));
    altitude = p / cos(lat_rad) - N;

    latitude_deg = lat_rad * 180.0 / M_PI;
    longitude_deg = lon_rad * 180.0 / M_PI;
}

namespace gazebo_plugins {
    class GazeboFixpositionPlugin : public gazebo::ModelPlugin {
    public:
        GazeboFixpositionPlugin() : gazebo::ModelPlugin() {
            std::cout << "Minimal Working Plugin constructed!\n";
        }

        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
            std::cout << "Minimal Working Plugin loaded!\n";
            model_ = _model;
            ros_node_ = gazebo_ros::Node::Get(_sdf);

            // Initialize publishers
            poi_gps_pub_ = ros_node_->create_publisher<sensor_msgs::msg::NavSatFix>("/fixposition/poi_navsatfix", 10);
            gps_sub_ = ros_node_->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/fixposition/vkrt_navsatfix", 10, 
                std::bind(&GazeboFixpositionPlugin::GpsCallback, this, std::placeholders::_1));
            // Check if poi_pose parameter exists
            if (_sdf->HasElement("poi_pose")) {
                auto poseStr = _sdf->Get<std::string>("poi_pose");
                std::istringstream iss(poseStr);
                double x, y, z, roll, pitch, yaw;
                iss >> x >> y >> z >> roll >> pitch >> yaw;
                poi_pose_.Pos() = ignition::math::Vector3d(x, y, z);
                poi_pose_.Rot() = ignition::math::Quaterniond(roll, pitch, yaw);
            } else {
                poi_pose_.Set(ignition::math::Vector3d(0, 0, 0), ignition::math::Quaterniond(0, 0, 0));
            }

            // Connect to update event
            update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                std::bind(&GazeboFixpositionPlugin::OnUpdate, this, std::placeholders::_1));
        }

        void GpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            last_gps_latitude = msg->latitude;
            last_gps_longitude = msg->longitude;
            last_gps_altitude = msg->altitude;
            got_gps = true;
        }

    private:
        void OnUpdate(const gazebo::common::UpdateInfo& info) {
            if (got_gps) {
                UpdatePOIPosition();
            }
        }

void UpdatePOIPosition() {
    gazebo::physics::LinkPtr link = model_->GetLink("Fixposition_upper_case");

    // Check if the link was found
    if (!link) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Link 'Fixposition_upper_case' not found in the model. Please check the link name.");
        return;
    }

    auto fixposition_upper_case_pose = link->WorldPose();

    // Create rotation matrix for the upper frame
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::Quaterniond(
        fixposition_upper_case_pose.Rot().W(),
        fixposition_upper_case_pose.Rot().X(),
        fixposition_upper_case_pose.Rot().Y(),
        fixposition_upper_case_pose.Rot().Z()
    ).toRotationMatrix();

    // Calculate relative position of POI in the world frame
    Eigen::Vector3d relative_position_eigen(poi_pose_.Pos().X(), poi_pose_.Pos().Y(), poi_pose_.Pos().Z());
    Eigen::Vector3d position_world = rotation_matrix * relative_position_eigen +
                                     Eigen::Vector3d(fixposition_upper_case_pose.Pos().X(),
                                                     fixposition_upper_case_pose.Pos().Y(),
                                                     fixposition_upper_case_pose.Pos().Z());

    RCLCPP_INFO(ros_node_->get_logger(), "Position World (X, Y, Z): (%f, %f, %f)", 
                position_world.x(), position_world.y(), position_world.z());

    // Convert world coordinates to ECEF
    Eigen::Matrix3d enu_to_ecef_rotation = RotEnuEcef(deg2rad(last_gps_latitude), deg2rad(last_gps_longitude));
    Eigen::Vector3d delta_ecef = enu_to_ecef_rotation * position_world;

    // Print delta_ecef to verify transformation accuracy
    RCLCPP_INFO(ros_node_->get_logger(), "Delta ECEF (X, Y, Z): (%f, %f, %f)", delta_ecef.x(), delta_ecef.y(), delta_ecef.z());

    // Calculate ECEF base from the "last GPS" location of vkrt_navsatfix
    geometry_msgs::msg::Point ecef_base = ConvertToECEF(last_gps_latitude, last_gps_longitude, last_gps_altitude);
    Eigen::Vector3d ecef_base_vec(ecef_base.x, ecef_base.y, ecef_base.z);

    // Final ECEF of POI
    Eigen::Vector3d ecef_poi = ecef_base_vec + delta_ecef;

    RCLCPP_INFO(ros_node_->get_logger(), "ECEF POI (X, Y, Z): (%f, %f, %f)", 
                ecef_poi.x(), ecef_poi.y(), ecef_poi.z());

    // Convert final ECEF to latitude, longitude, altitude
    double poi_latitude = 0.0, poi_longitude = 0.0, poi_altitude = 0.0;
    ECEFToLLA(ecef_poi.x(), ecef_poi.y(), ecef_poi.z(), poi_latitude, poi_longitude, poi_altitude);

    // Publish POI GPS coordinates
    sensor_msgs::msg::NavSatFix poi_gps_msg;
    poi_gps_msg.header.stamp = ros_node_->now();
    poi_gps_msg.header.frame_id = "FP_POI";
    poi_gps_msg.latitude = poi_latitude;
    poi_gps_msg.longitude = poi_longitude;
    poi_gps_msg.altitude = poi_altitude;

    poi_gps_pub_->publish(poi_gps_msg);

    // Print the final latitude, longitude, and altitude for additional verification
    RCLCPP_INFO(ros_node_->get_logger(), "POI Latitude: %f, Longitude: %f, Altitude: %f",
                poi_latitude, poi_longitude, poi_altitude);
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

        // Member variables
        gazebo_ros::Node::SharedPtr ros_node_;
        gazebo::event::ConnectionPtr update_connection_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr poi_gps_pub_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

        gazebo::physics::ModelPtr model_;
        ignition::math::Pose3d poi_pose_;
        bool got_gps = false;
        double last_gps_latitude = 0.0;
        double last_gps_longitude = 0.0;
        double last_gps_altitude = 0.0;
    };

    GZ_REGISTER_MODEL_PLUGIN(GazeboFixpositionPlugin)
} // namespace gazebo_plugins
