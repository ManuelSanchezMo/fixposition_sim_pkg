// gazebo_ros_fixposition.cpp

#include "fixposition_sensor_pkg/gazebo_ros_fixposition.hpp"
#include "fixposition_sensor_pkg/gazebo_fixposition_helper.hpp"
#include <gazebo/common/SphericalCoordinates.hh>

namespace gazebo_plugins {

GZ_REGISTER_MODEL_PLUGIN(GazeboFixpositionPlugin)

GazeboFixpositionPlugin::GazeboFixpositionPlugin()
    : gazebo::ModelPlugin(),
      update_period_(0, 100000000),         // 0.1 second
      publish_period_fusion_(0, 100000000), // 0.1 second
      last_publish_time_fusion_(0, 0),
      last_update_time_(0, 0),
      fixposition_helper_(),
      initialPoseSet(false),
      got_gps(false)
{
    std::cout << "[GazeboFixpositionPlugin] constructed!" << std::endl;
}

void GazeboFixpositionPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    std::cout << "[GazeboFixpositionPlugin] loaded!" << std::endl;

    // Store the model pointer
    model_ = _model;
    // Create a ROS 2 node
    ros_node_ = gazebo_ros::Node::Get(_sdf);
    // SDF parameters
    double default_update_frequency = 10.0;  // Example: 10 Hz
    if (_sdf->HasElement("update_frequency")) {
        default_update_frequency = _sdf->Get<double>("update_frequency");
    }
    update_period_ = gazebo::common::Time(1.0 / default_update_frequency);

    double fusion_publish_rate = 10.0;
    if (_sdf->HasElement("fusion_publish_rate")) {
        fusion_publish_rate = _sdf->Get<double>("fusion_publish_rate");
    }
    publish_period_fusion_ = gazebo::common::Time(1.0 / fusion_publish_rate);
    // Create publishers (custom Fixposition + standard ROS 2 messages)
    const gazebo_ros::QoS & qos = ros_node_->get_qos();

    odom_enu_pub_ = ros_node_->create_publisher<fixposition_msgs::msg::ODOMENU>(
        "/fixposition/fpa/odomenu",
        qos.get_publisher_qos("/fixposition/fpa/odomenu", rclcpp::SensorDataQoS().reliable()));

    odometry_pub_ = ros_node_->create_publisher<fixposition_msgs::msg::ODOMETRY>(
        "/fixposition/fpa/odometry",
        qos.get_publisher_qos("/fixposition/fpa/odometry", rclcpp::SensorDataQoS().reliable()));

    odom_status_pub_ = ros_node_->create_publisher<fixposition_msgs::msg::ODOMSTATUS>(
        "/fixposition/fpa/odomstatus",
        qos.get_publisher_qos("/fixposition/odomstatus", rclcpp::SensorDataQoS().reliable()));

    odom_sh_pub_ = ros_node_->create_publisher<fixposition_msgs::msg::ODOMSH>(
        "/fixposition/fpa/odomsh",
        qos.get_publisher_qos("/fixposition/fpa/odomsh", rclcpp::SensorDataQoS().reliable()));

    sh_odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>("/fixposition/odometry_smooth", 10); 


    ecef_odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>("/fixposition/odometry_ecef", 10);
    enu_odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>("/fixposition/odometry_enu", 10);

    imu_ypr_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Vector3>("/fixposition/imu_ypr", 10);
    ypr_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Vector3>("/fixposition/ypr", 10);
    poi_imu_pub_ = ros_node_->create_publisher<sensor_msgs::msg::Imu>("/fixposition/poiimu", 10);
    // TF broadcasters
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(ros_node_);

    // Subscribe to GPS data
    gps_sub_ = ros_node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/fixposition/poi_virtual_gps", 10,
        std::bind(&GazeboFixpositionPlugin::GpsCallback, this, std::placeholders::_1));

    llh_pub_ = ros_node_->create_publisher<fixposition_msgs::msg::LLH>("/fixposition/fpa/llh", 10);



    // Initialize drift to zero by default
    driftX_ = 0.0;
    driftY_ = 0.0;
    driftZ_ = 0.0;

    //Service to reset poish drift
    reset_poish_service_ = ros_node_->create_service<std_srvs::srv::Trigger>(
        "/fixposition/reset_poish",
        std::bind(&GazeboFixpositionPlugin::ResetPOISHDrift, this,
                  std::placeholders::_1,
                  std::placeholders::_2));
    // Example static transforms
    {
        // FP_VRTK -> FP_CAM
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

        // FP_POI -> FP_VRTK
        geometry_msgs::msg::TransformStamped tf_poi_vrtk;
        tf_poi_vrtk.header.frame_id = "FP_POI";
        tf_poi_vrtk.child_frame_id = "FP_VRTK";
        tf_poi_vrtk.transform.translation.x = 0.0;
        tf_poi_vrtk.transform.translation.y = 0.0;
        tf_poi_vrtk.transform.translation.z = 0.0;
        tf_poi_vrtk.transform.rotation.x = 0.0;
        tf_poi_vrtk.transform.rotation.y = 0.0;
        tf_poi_vrtk.transform.rotation.z = 0.0;
        tf_poi_vrtk.transform.rotation.w = 1.0;
        static_tf_broadcaster_->sendTransform(tf_poi_vrtk);
    }

    // Connect to Gazebo's update event
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboFixpositionPlugin::OnUpdate, this, std::placeholders::_1));
}

// GPS callback that republishes in the LLH format
void GazeboFixpositionPlugin::GpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // 1. Store values internally if you need them
    last_gps_latitude  = msg->latitude;
    last_gps_longitude = msg->longitude;
    last_gps_altitude  = msg->altitude;
    got_gps = true;

    // 2. Create LLH message
    fixposition_msgs::msg::LLH llh_msg;

    // 3. Fill header
    llh_msg.header.stamp    = msg->header.stamp;   
    llh_msg.header.frame_id = msg->header.frame_id;  // or your preferred frame_id

    // 4. Fill "pose_frame" (string) 
    //    (depending on how you want to describe the coordinate representation)
    llh_msg.pose_frame = "pose"; // or "quaternion", "ENU", etc. if relevant

    // 5. Fill position
    //    - lat, lon in degrees, alt in meters
    llh_msg.position.x = msg->latitude;    // latitude [deg]
    llh_msg.position.y = msg->longitude;   // longitude [deg]
    llh_msg.position.z = msg->altitude;    // height [m above ellipsoid?]

    // 6. Fill covariance (9 elements)
    //    NavSatFix::position_covariance is also 9 elements, typically describing 
    //    the lat/lon/alt uncertainties. If you want ENU, you might need a transform.
    //    For simplicity, let's directly copy the array:
    for (size_t i = 0; i < 9; i++) {
        llh_msg.covariance[i] = msg->position_covariance[i];
    }

    // 7. Publish LLH
    llh_pub_->publish(llh_msg);
}
void GazeboFixpositionPlugin::OnUpdate(const gazebo::common::UpdateInfo& info) {
    auto current_time = info.simTime;
    auto elapsed_time = current_time - last_update_time_;

    if (elapsed_time >= update_period_) {
        // Wait for the first GPS fix
        if (!initialPoseSet && got_gps) {
            auto link = model_->GetLink("POI_virtual_frame");
            if (!link) {
                RCLCPP_ERROR(ros_node_->get_logger(),
                             "Link 'POI_virtual_frame' not found in the model!");
                return;
            }
            // Store link's current pose as our reference
            initialPose = link->WorldPose();
            RCLCPP_INFO(ros_node_->get_logger(),
                "Initial Pose -> (%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f)",
                initialPose.Pos().X(), initialPose.Pos().Y(), initialPose.Pos().Z(),
                initialPose.Rot().W(), initialPose.Rot().X(), initialPose.Rot().Y(), initialPose.Rot().Z());

            // Publish ENU transform once
            PublishENUFrame();

            initialPoseSet = true;
        }

        if (initialPoseSet && got_gps) {
            UpdateOdometryWorld();
        }
        last_update_time_ = current_time;
    }

    // Check fusion publish rate
    auto fusion_elapsed = info.simTime - last_publish_time_fusion_;
    if (fusion_elapsed >= publish_period_fusion_) {
        if (initialPoseSet && got_gps) {
            PublishOdometryAndTFAndFusionMessages();
        }
        last_publish_time_fusion_ = info.simTime;
    }
}

// Publish ENU frame transform (ECEF->FP_ENU0) once
void GazeboFixpositionPlugin::PublishENUFrame() {
    geometry_msgs::msg::TransformStamped enu_tf;
    enu_tf.header.stamp = ros_node_->now();
    enu_tf.header.frame_id = "ECEF";
    enu_tf.child_frame_id = "FP_ENU0";

    auto ecefPose = fixposition_helper_.ConvertToECEF(
        last_gps_latitude, last_gps_longitude, last_gps_altitude);

    enu_tf.transform.translation.x = ecefPose.x;
    enu_tf.transform.translation.y = ecefPose.y;
    enu_tf.transform.translation.z = ecefPose.z;

    Eigen::Matrix3d enuToEcefRotation = fixposition_helper_.RotEnuEcef(
        GazeboFixpositionHelper::deg2rad(last_gps_latitude),
        GazeboFixpositionHelper::deg2rad(last_gps_longitude));
    auto enuToEcefQuat = fixposition_helper_.ConvertMatrixToQuaternion(enuToEcefRotation);
    enu_tf.transform.rotation = enuToEcefQuat;

    static_tf_broadcaster_->sendTransform(enu_tf);
}

void GazeboFixpositionPlugin::UpdateOdometryWorld() {
    auto link = model_->GetLink("POI_virtual_frame");
    if (!link) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Link 'POI_virtual_frame' not found!");
        return;
    }
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = ros_node_->now();          
    imu_msg.header.frame_id = "FP_IMU";               

    auto pose = link->WorldPose();
    imu_msg.orientation.x = pose.Rot().X();
    imu_msg.orientation.y = pose.Rot().Y();
    imu_msg.orientation.z = pose.Rot().Z();
    imu_msg.orientation.w = pose.Rot().W();
 
    auto ang_vel = link->RelativeAngularVel(); 
    imu_msg.angular_velocity.x = ang_vel.X();
    imu_msg.angular_velocity.y = ang_vel.Y();
    imu_msg.angular_velocity.z = ang_vel.Z();

    auto lin_acc = link->RelativeLinearAccel(); 
    imu_msg.linear_acceleration.x = lin_acc.X();
    imu_msg.linear_acceleration.y = lin_acc.Y();
    imu_msg.linear_acceleration.z = lin_acc.Z();

    poi_imu_pub_->publish(imu_msg);
    auto currentLinkPose = link->WorldPose();
    auto ecefPose = fixposition_helper_.ConvertToECEF(
        last_gps_latitude, last_gps_longitude, last_gps_altitude);
    Eigen::Vector3d ecefBase(ecefPose.x, ecefPose.y, ecefPose.z);

    Eigen::Matrix3d enuToEcef = fixposition_helper_.RotEnuEcef(
        GazeboFixpositionHelper::deg2rad(last_gps_latitude),
        GazeboFixpositionHelper::deg2rad(last_gps_longitude));

    // Subtract initial to get ENU
    ignition::math::Pose3d enuPose = TransformToENUFrame(currentLinkPose, initialPose);

    // ENU -> ECEF
    Eigen::Vector3d enuPos(enuPose.Pos().X(), enuPose.Pos().Y(), enuPose.Pos().Z());
    Eigen::Vector3d ecefPos = enuToEcef * enuPos + ecefBase;

    // Orientation
    Eigen::Quaterniond q_current(
        currentLinkPose.Rot().W(),
        currentLinkPose.Rot().X(),
        currentLinkPose.Rot().Y(),
        currentLinkPose.Rot().Z());
    Eigen::Matrix3d rot_current = q_current.toRotationMatrix();
    Eigen::Matrix3d rot_ecef = enuToEcef * rot_current;
    Eigen::Quaterniond ecefQuat(rot_ecef);

    // Fill ECEF odom
    odom_ecef.header.stamp = ros_node_->now();
    odom_ecef.header.frame_id = "ECEF";
    odom_ecef.child_frame_id = "FP_POI";
    odom_ecef.pose.pose.position.x = ecefPos.x();
    odom_ecef.pose.pose.position.y = ecefPos.y();
    odom_ecef.pose.pose.position.z = ecefPos.z();
    odom_ecef.pose.pose.orientation.x = ecefQuat.x();
    odom_ecef.pose.pose.orientation.y = ecefQuat.y();
    odom_ecef.pose.pose.orientation.z = ecefQuat.z();
    odom_ecef.pose.pose.orientation.w = ecefQuat.w();

    // Add velocity if needed
    odom_ecef.twist.twist.linear.x  = link->RelativeLinearVel().X();
    odom_ecef.twist.twist.angular.z = link->RelativeAngularVel().Z();

    // Fill ENU odom
    odom_enu.header.stamp = ros_node_->now();
    odom_enu.header.frame_id = "FP_ENU0";
    odom_enu.child_frame_id = "FP_POI";
    odom_enu.pose.pose.position.x = enuPose.Pos().X();
    odom_enu.pose.pose.position.y = enuPose.Pos().Y();
    odom_enu.pose.pose.position.z = enuPose.Pos().Z();
    odom_enu.pose.pose.orientation.x = enuPose.Rot().X();
    odom_enu.pose.pose.orientation.y = enuPose.Rot().Y();
    odom_enu.pose.pose.orientation.z = enuPose.Rot().Z();
    odom_enu.pose.pose.orientation.w = enuPose.Rot().W();

    odom_enu.twist.twist.linear.x  = link->RelativeLinearVel().X();
    odom_enu.twist.twist.angular.z = link->RelativeAngularVel().Z();
}

ignition::math::Pose3d GazeboFixpositionPlugin::TransformToENUFrame(
    const ignition::math::Pose3d& currentPose,
    const ignition::math::Pose3d& referencePose)
{
    ignition::math::Pose3d transformedPose = currentPose;
    transformedPose.Pos() = currentPose.Pos() - referencePose.Pos();
    return transformedPose;
}

void GazeboFixpositionPlugin::PublishOdometryAndTFAndFusionMessages() {
    //
    // 1) We define a single set of random noise for POI,
    //    and a single accumulated drift for POISH,
    //    so the odometry + transforms remain consistent.
    //
    static std::default_random_engine local_gen(42);
    static std::normal_distribution<double> noise_dist(0.0, 0.01); // e.g. 1 cm stdev

    // The base ECEF pose from odom_ecef
    double baseX = odom_ecef.pose.pose.position.x;
    double baseY = odom_ecef.pose.pose.position.y;
    double baseZ = odom_ecef.pose.pose.position.z;

    double noiseX = noise_dist(local_gen);
    double noiseY = noise_dist(local_gen);
    double noiseZ = noise_dist(local_gen);

    // "Noisy" POI
    double poiX = baseX + noiseX;
    double poiY = baseY + noiseY;
    double poiZ = baseZ + noiseZ;

    // For orientation, we can keep the same or add random orientation noise if needed.
    // We'll keep it identical for demonstration:
    auto poiOrient = odom_ecef.pose.pose.orientation;



    driftX_ += 0.002;  // 2 mm each publish cycle
    driftY_ += 0.001;  
    // driftZ_ += ?
    // POISH final position (relative to ECEF)
    double poishX = poiX + driftX_;
    double poishY = poiY + driftY_;
    double poishZ = poiZ + driftZ_;

    //
    // 2) Fill + publish the standard ROS odometry: ECEF & ENU
    //    They remain the "base" odom messages. We'll override ODOM_ECEF position w/ our new noisy coords.
    //
    ecef_odom_pub_->publish(odom_ecef);
    enu_odom_pub_->publish(odom_enu);

    //
    // 3) IMU / YPR if needed
    //
    auto link = model_->GetLink("POI_virtual_frame");
    if (link) {
        PublishIMUAndYPR(link->WorldPose());
    }

    //
    // 4) Publish transforms with the same noise/drift
    //

    // (A) ECEF -> FP_POI (with the same noise as above)
    geometry_msgs::msg::TransformStamped tf_ecef_poi;
    tf_ecef_poi.header.stamp = ros_node_->now();
    tf_ecef_poi.header.frame_id = "ECEF";
    tf_ecef_poi.child_frame_id = "FP_POI";
    tf_ecef_poi.transform.translation.x = poiX;
    tf_ecef_poi.transform.translation.y = poiY;
    tf_ecef_poi.transform.translation.z = poiZ;
    tf_ecef_poi.transform.rotation = poiOrient;
    tf_broadcaster_->sendTransform(tf_ecef_poi);

    // (B) FP_POI -> FP_POISH (with the same drift)
    geometry_msgs::msg::TransformStamped tf_poi_poish;
    tf_poi_poish.header.stamp = ros_node_->now();
    tf_poi_poish.header.frame_id = "FP_POI";
    tf_poi_poish.child_frame_id = "FP_POISH";
    tf_poi_poish.transform.translation.x = driftX_;
    tf_poi_poish.transform.translation.y = driftY_;
    tf_poi_poish.transform.translation.z = driftZ_;
    // Identity orientation or add drift as well if desired
    tf_poi_poish.transform.rotation.x = 0.0;
    tf_poi_poish.transform.rotation.y = 0.0;
    tf_poi_poish.transform.rotation.z = 0.0;
    tf_poi_poish.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf_poi_poish);

    //
    // 5) Publish the Fixposition Fusion messages (with consistent noise/drift)
    //
    PublishFusionMessagesConsistentNoiseDrift(poiX, poiY, poiZ, driftX_, driftY_, driftZ_);
}

void GazeboFixpositionPlugin::PublishIMUAndYPR(const ignition::math::Pose3d& pose) {
    auto rpy = pose.Rot().Euler();
    geometry_msgs::msg::Vector3 imu_ypr_msg;
    imu_ypr_msg.x = 0.0;  
    imu_ypr_msg.y = rpy.Y(); 
    imu_ypr_msg.z = rpy.X(); 
    imu_ypr_pub_->publish(imu_ypr_msg);

    geometry_msgs::msg::Vector3 ypr_msg;
    ypr_msg.x = rpy.Z(); // yaw
    ypr_msg.y = rpy.Y(); // pitch
    ypr_msg.z = rpy.X(); // roll
    ypr_pub_->publish(ypr_msg);
}

/**
 * @brief PublishFusionMessagesConsistentNoiseDrift
 * 
 * Here we replicate the exact same POI noise & POISH drift in the custom Fixposition messages.
 */
void GazeboFixpositionPlugin::PublishFusionMessagesConsistentNoiseDrift(
    double poiX, double poiY, double poiZ,
    double driftX_, double driftY_, double driftZ_)
{
    rclcpp::Time current_time = ros_node_->now();

    // Covariances (example)
    std::array<double, 36> pose_cov = {
        0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.1
    };
    std::array<double, 36> twist_cov = {
        0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.05, 0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.05, 0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.05, 0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.05
    };

    //
    // 1) ODOMENU (ENU)
    //
    fixposition_msgs::msg::ODOMENU odom_enu_msg;
    odom_enu_msg.header.stamp = current_time;
    odom_enu_msg.header.frame_id = "FP_ENU0";
    odom_enu_msg.pose_frame = "FP_ENU0";
    odom_enu_msg.kin_frame = "FP_ENU0";

    odom_enu_msg.pose =
        fixposition_helper_.ConvertToPoseWithCovariance(odom_enu.pose.pose, pose_cov);
    odom_enu_msg.velocity =
        fixposition_helper_.ConvertToTwistWithCovariance(odom_enu.twist.twist, twist_cov);
    odom_enu_pub_->publish(odom_enu_msg);

    //
    // 2) ODOMETRY (ECEF) => "POI" with the same random jump
    //
    fixposition_msgs::msg::ODOMETRY odometry_msg;
    odometry_msg.header.stamp = current_time;
    odometry_msg.header.frame_id = "ECEF";
    odometry_msg.pose_frame = "ECEF";
    odometry_msg.kin_frame = "ECEF";
    odometry_msg.version = "1.0.0";

    // We build a PoseWithCov where we place the POI noise in position
    geometry_msgs::msg::PoseWithCovariance poiPoseWithCov;
    poiPoseWithCov.pose.position.x = poiX;
    poiPoseWithCov.pose.position.y = poiY;
    poiPoseWithCov.pose.position.z = poiZ;

    // Orientation can come from odom_ecef
    poiPoseWithCov.pose.orientation = odom_ecef.pose.pose.orientation;
    poiPoseWithCov.covariance = pose_cov;

    odometry_msg.pose = poiPoseWithCov;

    // Velocity
    geometry_msgs::msg::TwistWithCovariance poiTwistCov =
        fixposition_helper_.ConvertToTwistWithCovariance(odom_ecef.twist.twist, twist_cov);
    odometry_msg.velocity = poiTwistCov;

    odometry_pub_->publish(odometry_msg);

    //
    // 3) ODOMSTATUS
    //
    fixposition_msgs::msg::ODOMSTATUS odom_status_msg;
    odom_status_msg.header.stamp = current_time;
    odom_status_pub_->publish(odom_status_msg);

    //
    // 4) ODOMSH => "Smooth odometry" with same drift
    //
    fixposition_msgs::msg::ODOMSH odom_sh_msg;
    odom_sh_msg.header.stamp = current_time;
    odom_sh_msg.header.frame_id = "ECEF";
    odom_sh_msg.pose_frame = "ECEF";
    odom_sh_msg.kin_frame = "ECEF";

    geometry_msgs::msg::PoseWithCovariance poishPoseWithCov;
    poishPoseWithCov.covariance = pose_cov;

    // POISH = POI + drift
    poishPoseWithCov.pose.position.x = poiX + driftX_;
    poishPoseWithCov.pose.position.y = poiY + driftY_;
    poishPoseWithCov.pose.position.z = poiZ + driftZ_;

    // Keep orientation from odom_ecef or use identity, as desired
    poishPoseWithCov.pose.orientation = odom_ecef.pose.pose.orientation;

    odom_sh_msg.pose = poishPoseWithCov;
    odom_sh_msg.velocity =
        fixposition_helper_.ConvertToTwistWithCovariance(odom_ecef.twist.twist, twist_cov);

    odom_sh_pub_->publish(odom_sh_msg);
    // -------------------------------------------------------------------------
    //  build and publish a nav_msgs::msg::Odometry for the same smooth odometry
    // -------------------------------------------------------------------------
    nav_msgs::msg::Odometry odom_sh_nav;
    odom_sh_nav.header.stamp = current_time;
    
    odom_sh_nav.header.frame_id = "ECEF";

    odom_sh_nav.child_frame_id = "FP_POISH";

    odom_sh_nav.pose = odom_sh_msg.pose; 

    // Twist
    odom_sh_nav.twist = odom_sh_msg.velocity; // same TwistWithCovariance

    // Publish nav_msgs Odometry for smooth odom
    sh_odom_pub_->publish(odom_sh_nav);
    }

void GazeboFixpositionPlugin::ResetPOISHDrift(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    // Reset drift to zero
    driftX_ = 0.0;
    driftY_ = 0.0;
    driftZ_ = 0.0;

    // Let the user know it's reset
    response->success = true;
    response->message = "POISH drift has been reset to zero.";
    std::cout << "[GazeboFixpositionPlugin] POISH drift reset to zero." << std::endl;
}


} // namespace gazebo_plugins
