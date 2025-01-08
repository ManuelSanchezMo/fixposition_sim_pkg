// gazebo_fixposition_helper.cpp

#include "fixposition_sensor_pkg/gazebo_fixposition_helper.hpp"
#include <cmath>
#include <algorithm>

namespace gazebo_plugins {

// Initialize constants
constexpr double GazeboFixpositionHelper::a;
constexpr double GazeboFixpositionHelper::e_squared;

// Constructor
GazeboFixpositionHelper::GazeboFixpositionHelper() {
    // Initialization if needed
}

// Convert degrees to radians
constexpr double GazeboFixpositionHelper::deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

// Convert Twist to TwistWithCovariance
geometry_msgs::msg::TwistWithCovariance GazeboFixpositionHelper::ConvertToTwistWithCovariance(
    const geometry_msgs::msg::Twist& twist,
    const std::array<double, 36>& covariance) const
{
    geometry_msgs::msg::TwistWithCovariance twist_with_covariance;
    twist_with_covariance.twist = twist;
    // Copy the covariance values
    std::copy(covariance.begin(), covariance.end(), twist_with_covariance.covariance.begin());
    return twist_with_covariance;
}

// Convert Pose to PoseWithCovariance
geometry_msgs::msg::PoseWithCovariance GazeboFixpositionHelper::ConvertToPoseWithCovariance(
    const geometry_msgs::msg::Pose& pose,
    const std::array<double, 36>& covariance) const
{
    geometry_msgs::msg::PoseWithCovariance pose_with_covariance;
    pose_with_covariance.pose = pose;
    // Copy the covariance values
    std::copy(covariance.begin(), covariance.end(), pose_with_covariance.covariance.begin());
    return pose_with_covariance;
}

// Convert geographic coordinates to ECEF
geometry_msgs::msg::Point GazeboFixpositionHelper::ConvertToECEF(double latitude, double longitude, double altitude) const {
    double lat_rad = deg2rad(latitude);
    double lon_rad = deg2rad(longitude);

    geometry_msgs::msg::Point ecef;
    double N = a / std::sqrt(1 - e_squared * std::sin(lat_rad) * std::sin(lat_rad));
    ecef.x = (N + altitude) * std::cos(lat_rad) * std::cos(lon_rad);
    ecef.y = (N + altitude) * std::cos(lat_rad) * std::sin(lon_rad);
    ecef.z = (N * (1 - e_squared) + altitude) * std::sin(lat_rad);

    return ecef;
}

// Convert ECEF to Latitude, Longitude, Altitude
void GazeboFixpositionHelper::ECEFToLLA(double x, double y, double z, double& latitude_deg, double& longitude_deg, double& altitude) const {
    double b = std::sqrt(a * a * (1 - e_squared));
    double ep = std::sqrt((a * a - b * b) / (b * b));
    double p = std::sqrt(x * x + y * y);
    double th = std::atan2(a * z, b * p);

    double lon_rad = std::atan2(y, x);
    double lat_rad = std::atan2(z + ep * ep * b * std::pow(std::sin(th), 3), p - e_squared * a * std::pow(std::cos(th), 3));
    double N = a / std::sqrt(1 - e_squared * std::sin(lat_rad) * std::sin(lat_rad));
    altitude = p / std::cos(lat_rad) - N;

    latitude_deg = lat_rad * 180.0 / M_PI;
    longitude_deg = lon_rad * 180.0 / M_PI;
}

// Rotation matrix from ENU to ECEF
Eigen::Matrix3d GazeboFixpositionHelper::RotEnuEcef(double lat_rad, double lon_rad) const {
    double sLat = std::sin(lat_rad);
    double cLat = std::cos(lat_rad);
    double sLon = std::sin(lon_rad);
    double cLon = std::cos(lon_rad);

    Eigen::Matrix3d R;
    R << -sLon, -sLat * cLon, cLat * cLon,
          cLon, -sLat * sLon, cLat * sLon,
          0,     cLat,         sLat;
    return R;
}

// Convert rotation matrix to Quaternion
geometry_msgs::msg::Quaternion GazeboFixpositionHelper::ConvertMatrixToQuaternion(const Eigen::Matrix3d& matrix) const {
    Eigen::Quaterniond quat(matrix);
    geometry_msgs::msg::Quaternion quaternion_msg;
    quaternion_msg.x = quat.x();
    quaternion_msg.y = quat.y();
    quaternion_msg.z = quat.z();
    quaternion_msg.w = quat.w();
    return quaternion_msg;
}

} // namespace gazebo_plugins

