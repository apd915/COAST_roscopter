/**
 * Converts FAST-LIO odometry into the estimator's NED frame, as a CG position measurement.
 *
 * FAST-LIO reports the pose of the LiDAR's internal IMU in "camera_init": that IMU's own frame
 * (x forward, y left, z up) at the moment FAST-LIO started. Three things separate that from what
 * the EKF needs:
 *
 *   1. Axes. FLU -> FRD is a 180 deg roll, F = diag(1, -1, -1), its own inverse. Assumes the LiDAR
 *      is mounted upright with its x axis along the body x axis.
 *   2. Origin and heading. camera_init is wherever the vehicle sat, pointing wherever it pointed
 *      and tilted however the ground tilted it. The EKF's frame is NED about the GPS origin with
 *      magnetometer heading. Neither is observable from the LiDAR alone, so this node ALIGNS once:
 *      after the vehicle has been armed for align_settle_s (the EKF only runs while armed), it
 *      takes the EKF pose nearest the odometry stamp and solves for the rotation and offset that
 *      map one onto the other. Nothing is published before that, so the EKF never sees an
 *      unaligned measurement.
 *   3. Lever arm. FAST-LIO tracks the LiDAR IMU, not the CG.
 *
 * Alignment inherits the EKF's attitude error at that instant. A heading error rotates the whole
 * LIO track about the alignment point; a tilt error makes it climb or sink with distance (1 deg
 * is 1.7 m per 100 m). Align while stationary on level-ish ground.
 */
#include <algorithm>
#include <deque>
#include <memory>
#include <vector>

#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <roscopter_msgs/msg/state.hpp>
#include <rosflight_msgs/msg/status.hpp>

namespace roscopter
{

class LioNedBridge : public rclcpp::Node
{
public:
  LioNedBridge()
      : Node("lio_ned_bridge")
  {
    // LiDAR IMU position relative to the CG, body FRD, metres. Zero until the mount is measured.
    this->declare_parameter("lever_arm_frd", std::vector<double>{0.0, 0.0, 0.0});
    // Seconds armed before aligning. Long enough for the EKF heading and tilt to settle.
    this->declare_parameter("align_settle_s", 3.0);

    const auto lever = this->get_parameter("lever_arm_frd").as_double_array();
    if (lever.size() != 3) {
      throw std::runtime_error("lever_arm_frd must have exactly 3 elements");
    }
    lever_frd_ = Eigen::Vector3d(lever[0], lever[1], lever[2]);
    align_settle_s_ = this->get_parameter("align_settle_s").as_double();

    flu_to_frd_ = Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lio/odom_ned", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 10, std::bind(&LioNedBridge::odom_callback, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<roscopter_msgs::msg::State>(
      "estimated_state", 10, std::bind(&LioNedBridge::state_callback, this, std::placeholders::_1));
    status_sub_ = this->create_subscription<rosflight_msgs::msg::Status>(
      "status", 1, std::bind(&LioNedBridge::status_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Lever arm (FRD): [%.3f, %.3f, %.3f] m. Aligning %.1f s after arming.",
                lever_frd_.x(), lever_frd_.y(), lever_frd_.z(), align_settle_s_);
  }

private:
  struct EkfPose
  {
    rclcpp::Time stamp;
    Eigen::Vector3d p_ned;
    Eigen::Matrix3d R_ned_body;
  };

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<roscopter_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Status>::SharedPtr status_sub_;

  Eigen::Vector3d lever_frd_;
  double align_settle_s_;
  Eigen::Matrix3d flu_to_frd_;

  std::deque<EkfPose> ekf_history_;
  bool armed_ = false;
  rclcpp::Time armed_since_;

  bool aligned_ = false;
  Eigen::Matrix3d R_align_;  // camera_init (FRD axes) -> NED
  Eigen::Vector3d t_align_;  // NED position of the camera_init origin

  void status_callback(const rosflight_msgs::msg::Status & msg)
  {
    if (msg.armed && !armed_) {
      armed_since_ = this->now();
    }
    armed_ = msg.armed;
  }

  void state_callback(const roscopter_msgs::msg::State & msg)
  {
    EkfPose pose;
    pose.stamp = msg.header.stamp;
    pose.p_ned = Eigen::Vector3d(msg.p_n, msg.p_e, msg.p_d);
    pose.R_ned_body =
      Eigen::Quaterniond(msg.quat.w, msg.quat.x, msg.quat.y, msg.quat.z).normalized().toRotationMatrix();
    ekf_history_.push_back(pose);

    // ~1 s at the estimator's 390 Hz; enough to bracket FAST-LIO's processing latency.
    while (ekf_history_.size() > 400) {
      ekf_history_.pop_front();
    }
  }

  const EkfPose * nearest_ekf_pose(const rclcpp::Time & stamp) const
  {
    const EkfPose * best = nullptr;
    double best_dt = std::numeric_limits<double>::infinity();
    for (const auto & pose : ekf_history_) {
      const double dt = std::abs((pose.stamp - stamp).seconds());
      if (dt < best_dt) {
        best_dt = dt;
        best = &pose;
      }
    }
    return best;
  }

  void odom_callback(const nav_msgs::msg::Odometry & msg)
  {
    const rclcpp::Time stamp(msg.header.stamp);

    // LiDAR IMU pose in camera_init, re-expressed with FRD axes so it composes with NED.
    const Eigen::Vector3d p_flu(msg.pose.pose.position.x, msg.pose.pose.position.y,
                                msg.pose.pose.position.z);
    const Eigen::Matrix3d R_flu =
      Eigen::Quaterniond(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                         msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
        .normalized().toRotationMatrix();
    const Eigen::Vector3d p_init = flu_to_frd_ * p_flu;
    const Eigen::Matrix3d R_init_body = flu_to_frd_ * R_flu * flu_to_frd_;

    if (!aligned_ && !try_align(stamp, p_init, R_init_body)) {
      return;
    }

    const Eigen::Matrix3d R_ned_body = R_align_ * R_init_body;
    const Eigen::Vector3d p_lidar_ned = R_align_ * p_init + t_align_;
    const Eigen::Vector3d p_cg_ned = p_lidar_ned - R_ned_body * lever_frd_;
    const Eigen::Quaterniond q_ned_body(R_ned_body);

    nav_msgs::msg::Odometry out;
    out.header.stamp = msg.header.stamp;
    out.header.frame_id = "NED";
    out.child_frame_id = "body";
    out.pose.pose.position.x = p_cg_ned.x();
    out.pose.pose.position.y = p_cg_ned.y();
    out.pose.pose.position.z = p_cg_ned.z();
    out.pose.pose.orientation.w = q_ned_body.w();
    out.pose.pose.orientation.x = q_ned_body.x();
    out.pose.pose.orientation.y = q_ned_body.y();
    out.pose.pose.orientation.z = q_ned_body.z();
    odom_pub_->publish(out);
  }

  bool try_align(const rclcpp::Time & stamp, const Eigen::Vector3d & p_init,
                 const Eigen::Matrix3d & R_init_body)
  {
    if (!armed_ || (this->now() - armed_since_).seconds() < align_settle_s_) {
      return false;
    }
    const EkfPose * ekf = nearest_ekf_pose(stamp);
    if (ekf == nullptr) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Armed but no estimated_state received; cannot align LIO.");
      return false;
    }

    // Choose R_align, t_align so that, at this instant, the LIO pose maps exactly onto the
    // EKF pose: R_align * R_init_body = R_ned_body, and the LiDAR sits at p_cg + R * lever.
    R_align_ = ekf->R_ned_body * R_init_body.transpose();
    t_align_ = ekf->p_ned + ekf->R_ned_body * lever_frd_ - R_align_ * p_init;
    aligned_ = true;

    // ZYX angles taken directly from the matrix. Eigen's eulerAngles() restricts its first angle
    // to [0, pi] and so reports a near-identity rotation as (180, -180, -180).
    const double deg = 180.0 / M_PI;
    const double yaw = std::atan2(R_align_(1, 0), R_align_(0, 0)) * deg;
    const double pitch = -std::asin(std::clamp(R_align_(2, 0), -1.0, 1.0)) * deg;
    const double roll = std::atan2(R_align_(2, 1), R_align_(2, 2)) * deg;
    RCLCPP_INFO(this->get_logger(),
                "LIO aligned to EKF (pose %.3f s from odometry stamp). camera_init -> NED: "
                "yaw %.2f pitch %.2f roll %.2f deg, origin [%.2f, %.2f, %.2f] m NED.",
                std::abs((ekf->stamp - stamp).seconds()), yaw, pitch, roll,
                t_align_.x(), t_align_.y(), t_align_.z());
    return true;
  }
};

} // namespace roscopter

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roscopter::LioNedBridge>());
  rclcpp::shutdown();
  return 0;
}
