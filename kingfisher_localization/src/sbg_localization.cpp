// Copyright 2026 Luis Batista
//
// Licensed under the BSD License; see the package.xml for details.

// SBG Ellipse (driver in NED) -> the localization contract, the same topics as the sim (VRX).
//
// * sensors/imu/imu/data (sensor_msgs/Imu, imu_frame): orientation of imu_link in ENU,
//   rates and accelerations in FLU, on every sbg/imu_data;
// * sensors/gps/gps/fix (sensor_msgs/NavSatFix, gps_frame), on every sbg/gps_pos;
// * odom (nav_msgs/Odometry) and its TF odom -> base_frame, on every sbg/ekf_nav.
//   Position is ENU about the site datum (map = odom), twist is in base_frame (FLU).
//
// Without a valid position (indoors) odom is still published: the position is held at the last
// valid one (the datum if none) and the velocity is 0, both with a large variance.
//
// The imu_link position in base_frame comes from TF (the bringup's robot_state_publisher), looked
// up once: odom waits for it, the IMU and GPS topics don't.

#include <GeographicLib/LocalCartesian.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sbg_driver/msg/sbg_ekf_nav.hpp"
#include "sbg_driver/msg/sbg_ekf_quat.hpp"
#include "sbg_driver/msg/sbg_gps_pos.hpp"
#include "sbg_driver/msg/sbg_imu_data.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace kingfisher_localization
{

using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;

// ENU <- NED, and FRD <- FLU (pi about x); tf2 order is (x, y, z, w)
const tf2::Quaternion kEnuNed(M_SQRT1_2, M_SQRT1_2, 0.0, 0.0);
const tf2::Quaternion kFrdFlu(1.0, 0.0, 0.0, 0.0);

// FRD vector -> FLU
tf2::Vector3 flu(const geometry_msgs::msg::Vector3 & v)
{
  return {v.x, -v.y, -v.z};
}

// SbgGpsPosStatus.type -> NavSatStatus.status
int8_t gps_status(uint8_t type)
{
  switch (type) {
    case 0: return NavSatStatus::STATUS_NO_FIX;
    case 4: return NavSatStatus::STATUS_SBAS_FIX;
    case 6: case 7: return NavSatStatus::STATUS_GBAS_FIX;
    default: return NavSatStatus::STATUS_FIX;
  }
}

// ENU tangent plane at the datum of a config/sites file
GeographicLib::LocalCartesian load_site(const std::string & path)
{
  const YAML::Node d = YAML::LoadFile(path)["datum"];
  return GeographicLib::LocalCartesian(
    d["latitude"].as<double>(), d["longitude"].as<double>(), d["altitude"].as<double>());
}

// Convert the SBG driver's raw NED messages to the ENU/FLU contract topics.
class SbgLocalization : public rclcpp::Node
{
public:
  SbgLocalization()
  : Node("sbg_localization"),
    enu_(load_site(declare_parameter("site_file", std::string()))),
    odom_frame_(declare_parameter("odom_frame", std::string("odom"))),
    base_frame_(declare_parameter("base_frame", std::string("base_link"))),
    imu_frame_(declare_parameter("imu_frame", std::string("imu_link"))),
    gps_frame_(declare_parameter("gps_frame", std::string("gps_link"))),
    // m^2 and (m/s)^2, for a position or velocity the SBG doesn't have
    unknown_variance_(declare_parameter("unknown_variance", 1e6)),
    tf_buffer_(get_clock()),
    tf_listener_(std::make_unique<tf2_ros::TransformListener>(tf_buffer_, this)),
    tf_broadcaster_(this)
  {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("sensors/imu/imu/data", 10);
    fix_pub_ = create_publisher<NavSatFix>("sensors/gps/gps/fix", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    quat_sub_ = create_subscription<sbg_driver::msg::SbgEkfQuat>(
      "sbg/ekf_quat", 10, [this](const sbg_driver::msg::SbgEkfQuat & msg) {on_quat(msg);});
    imu_sub_ = create_subscription<sbg_driver::msg::SbgImuData>(
      "sbg/imu_data", 10, [this](const sbg_driver::msg::SbgImuData & msg) {on_imu(msg);});
    gps_sub_ = create_subscription<sbg_driver::msg::SbgGpsPos>(
      "sbg/gps_pos", 10, [this](const sbg_driver::msg::SbgGpsPos & msg) {on_gps(msg);});
    nav_sub_ = create_subscription<sbg_driver::msg::SbgEkfNav>(
      "sbg/ekf_nav", 10, [this](const sbg_driver::msg::SbgEkfNav & msg) {on_nav(msg);});
  }

private:
  void on_quat(const sbg_driver::msg::SbgEkfQuat & msg)
  {
    tf2::Quaternion q_ned;
    tf2::fromMsg(msg.quaternion, q_ned);
    q_ = kEnuNed * q_ned * kFrdFlu;
    const auto & a = msg.accuracy;
    q_variance_ = {a.x * a.x, a.y * a.y, a.z * a.z};
  }

  void on_imu(const sbg_driver::msg::SbgImuData & msg)
  {
    omega_ = flu(msg.gyro);
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = msg.header.stamp;
    imu.header.frame_id = imu_frame_;
    if (!q_) {
      imu.orientation_covariance[0] = -1.0;
    } else {
      imu.orientation = tf2::toMsg(*q_);
      for (int i = 0; i < 3; ++i) {
        imu.orientation_covariance[4 * i] = q_variance_[i];
      }
    }
    imu.angular_velocity = tf2::toMsg(omega_);
    imu.linear_acceleration = tf2::toMsg(flu(msg.accel));
    imu_pub_->publish(imu);
  }

  void on_gps(const sbg_driver::msg::SbgGpsPos & msg)
  {
    NavSatFix fix;
    fix.header.stamp = msg.header.stamp;
    fix.header.frame_id = gps_frame_;
    fix.status.service = NavSatStatus::SERVICE_GPS;
    if (msg.status.status != 0) {  // not SOL_COMPUTED
      fix.status.status = NavSatStatus::STATUS_NO_FIX;
    } else {
      fix.status.status = gps_status(msg.status.type);
    }
    fix.latitude = msg.latitude;
    fix.longitude = msg.longitude;
    // Above mean sea level, like the site datum (and the sim's navsat at the water)
    fix.altitude = msg.altitude;
    const auto & acc = msg.position_accuracy;  // NED: x north, y east
    fix.position_covariance[0] = acc.y * acc.y;
    fix.position_covariance[4] = acc.x * acc.x;
    fix.position_covariance[8] = acc.z * acc.z;
    fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    fix_pub_->publish(fix);
  }

  // imu_frame's position in base_frame, once TF has it
  bool lookup_lever()
  {
    if (lever_) {
      return true;
    }
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_.lookupTransform(base_frame_, imu_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "no odom until TF has %s in %s: %s",
        imu_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
    lever_.emplace();
    tf2::fromMsg(t.transform.translation, *lever_);
    RCLCPP_INFO(
      get_logger(), "%s in %s: (%.3f, %.3f, %.3f)", imu_frame_.c_str(), base_frame_.c_str(),
      lever_->x(), lever_->y(), lever_->z());
    // Static: no need to keep listening to /tf
    tf_listener_.reset();
    return true;
  }

  void on_nav(const sbg_driver::msg::SbgEkfNav & msg)
  {
    if (!q_ || !lookup_lever()) {
      return;
    }
    const tf2::Quaternion & q = *q_;
    const tf2::Vector3 & lever = *lever_;  // m, the axes are assumed aligned (as in the URDF)
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg.header.stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    const double big = unknown_variance_;
    std::array<double, 3> position_variance{big, big, big};
    if (msg.status.position_valid) {
      double x, y, z;
      enu_.Forward(msg.latitude, msg.longitude, msg.altitude, x, y, z);
      // imu_link -> base_link
      position_ = tf2::Vector3(x, y, z) - tf2::quatRotate(q, lever);
      const auto & acc = msg.position_accuracy;  // NED: x north, y east
      position_variance = {acc.y * acc.y, acc.x * acc.x, acc.z * acc.z};
    }
    tf2::toMsg(position_, odom.pose.pose.position);
    odom.pose.pose.orientation = tf2::toMsg(q);
    for (int i = 0; i < 3; ++i) {
      odom.pose.covariance[7 * i] = position_variance[i];
      odom.pose.covariance[7 * (i + 3)] = q_variance_[i];
    }

    tf2::Vector3 v_base(0.0, 0.0, 0.0);
    std::array<double, 3> velocity_variance{big, big, big};
    if (msg.status.velocity_valid) {
      const auto & v = msg.velocity;  // NED
      // imu_link velocity, in the body; base_link = imu_link - omega x lever
      v_base = tf2::quatRotate(q.inverse(), tf2::Vector3(v.y, v.x, -v.z)) - omega_.cross(lever);
      const auto & acc = msg.velocity_accuracy;
      const double h = std::pow(std::max(acc.x, acc.y), 2);
      velocity_variance = {h, h, acc.z * acc.z};
    }
    odom.twist.twist.linear = tf2::toMsg(v_base);
    odom.twist.twist.angular = tf2::toMsg(omega_);
    for (int i = 0; i < 3; ++i) {
      odom.twist.covariance[7 * i] = velocity_variance[i];
    }
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = base_frame_;
    tf.transform.translation = tf2::toMsg(position_);
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_.sendTransform(tf);
  }

  GeographicLib::LocalCartesian enu_;
  std::string odom_frame_, base_frame_, imu_frame_, gps_frame_;
  double unknown_variance_;

  std::optional<tf2::Quaternion> q_;            // imu_link in ENU, from the last ekf_quat
  std::array<double, 3> q_variance_{};          // roll, pitch, yaw
  tf2::Vector3 omega_{0.0, 0.0, 0.0};           // rad/s, FLU, from the last imu_data
  tf2::Vector3 position_{0.0, 0.0, 0.0};        // base_frame in odom, the last valid one
  std::optional<tf2::Vector3> lever_;           // imu_frame in base_frame, from TF

  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sbg_driver::msg::SbgEkfQuat>::SharedPtr quat_sub_;
  rclcpp::Subscription<sbg_driver::msg::SbgImuData>::SharedPtr imu_sub_;
  rclcpp::Subscription<sbg_driver::msg::SbgGpsPos>::SharedPtr gps_sub_;
  rclcpp::Subscription<sbg_driver::msg::SbgEkfNav>::SharedPtr nav_sub_;
};

}  // namespace kingfisher_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kingfisher_localization::SbgLocalization>());
  rclcpp::shutdown();
  return 0;
}
