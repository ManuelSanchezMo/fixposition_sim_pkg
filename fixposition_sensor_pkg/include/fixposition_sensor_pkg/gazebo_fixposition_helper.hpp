// gazebo_fixposition_helper.hpp

#ifndef GAZEBO_FIXPOSITION_HELPER_HPP
#define GAZEBO_FIXPOSITION_HELPER_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>

namespace gazebo_plugins {

/**
 * @brief A helper class for performing various coordinate transformations and utility functions.
 */
class GazeboFixpositionHelper {
public:
    // Constructor
    GazeboFixpositionHelper();

    // Constants for WGS-84 Earth model
    static constexpr double a = 6378137.0;                 // Semi-major axis (meters)
    static constexpr double e_squared = 0.00669437999014;  // Eccentricity squared

    /**
     * @brief Converts degrees to radians.
     * @param degrees Angle in degrees.
     * @return Angle in radians.
     */
    static constexpr double deg2rad(double degrees);

    /**
     * @brief Converts a Twist message to TwistWithCovariance.
     * @param twist The Twist message.
     * @param covariance The covariance array (36 elements).
     * @return TwistWithCovariance message.
     */
    geometry_msgs::msg::TwistWithCovariance ConvertToTwistWithCovariance(
        const geometry_msgs::msg::Twist& twist,
        const std::array<double, 36>& covariance) const;

    /**
     * @brief Converts a Pose message to PoseWithCovariance.
     * @param pose The Pose message.
     * @param covariance The covariance array (36 elements).
     * @return PoseWithCovariance message.
     */
    geometry_msgs::msg::PoseWithCovariance ConvertToPoseWithCovariance(
        const geometry_msgs::msg::Pose& pose,
        const std::array<double, 36>& covariance) const;

    /**
     * @brief Converts geographic coordinates to ECEF.
     * @param latitude Latitude in degrees.
     * @param longitude Longitude in degrees.
     * @param altitude Altitude in meters.
     * @return Point in ECEF coordinates.
     */
    geometry_msgs::msg::Point ConvertToECEF(double latitude, double longitude, double altitude) const;

    /**
     * @brief Converts ECEF coordinates to Latitude, Longitude, Altitude.
     * @param x ECEF X coordinate.
     * @param y ECEF Y coordinate.
     * @param z ECEF Z coordinate.
     * @param latitude_deg Output latitude in degrees.
     * @param longitude_deg Output longitude in degrees.
     * @param altitude Output altitude in meters.
     */
    void ECEFToLLA(double x, double y, double z, double& latitude_deg, double& longitude_deg, double& altitude) const;

    /**
     * @brief Computes the rotation matrix from ENU to ECEF.
     * @param lat_rad Latitude in radians.
     * @param lon_rad Longitude in radians.
     * @return 3x3 rotation matrix.
     */
    Eigen::Matrix3d RotEnuEcef(double lat_rad, double lon_rad) const;

    /**
     * @brief Converts a rotation matrix to a geometry_msgs Quaternion.
     * @param matrix 3x3 rotation matrix.
     * @return Quaternion message.
     */
    geometry_msgs::msg::Quaternion ConvertMatrixToQuaternion(const Eigen::Matrix3d& matrix) const;

private:
    // Private members can be added here if needed in the future
};

} // namespace gazebo_plugins

#endif // GAZEBO_FIXPOSITION_HELPER_HPP

