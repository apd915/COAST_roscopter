/**
 * Stand-in for FAST-LIO in simulation: publishes /Odometry exactly as FAST-LIO does, from truth.
 *
 * What it reproduces, because lio_ned_bridge and the EKF depend on each of these:
 *   - Frame. Pose of the LiDAR's IMU in "camera_init", which is that IMU's own frame (x forward,
 *     y left, z up) at the moment the node started -- not NED, not gravity-aligned, not north-up.
 *   - Content. Pose only; FAST-LIO leaves twist at zero. Stamped at the end of the scan.
 *   - Rate and latency. One estimate per 10 Hz scan, published latency_s after its stamp.
 *   - Error. White noise per estimate (bench-measured jitter with the Mid-360S was ~2 cm) plus a
 *     drift random walk that only grows while the vehicle MOVES, as scan-matching drift does.
 *
 * What it does not: degeneracy (featureless scenes), loop closure, attitude drift, or failure.
 */
#include <deque>
#include <memory>
#include <random>
#include <vector>

#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rosflight_msgs/msg/sim_state.hpp"

namespace roscopter_sim {

class MockLioOdometry : public rclcpp::Node
{
public:
  MockLioOdometry()
      : Node("mock_lio_odometry")
  {
    this->declare_parameter("rate_hz", 10.0);
    this->declare_parameter("latency_s", 0.03);
    this->declare_parameter("position_noise_stdev", 0.02);  // m, per estimate
    // Drift random-walk stdev per sqrt(metre travelled), per axis. 0.05 gives ~0.5 m (1-sigma)
    // after 100 m -- about 0.5% of distance, a typical FAST-LIO figure outdoors.
    this->declare_parameter("drift_stdev_per_sqrt_m", 0.05);
    // Must match lio_ned_bridge's lever_arm_frd for the bridge to recover the CG exactly.
    this->declare_parameter("lever_arm_frd", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter("seed", 0);

    const auto lever = this->get_parameter("lever_arm_frd").as_double_array();
    if (lever.size() != 3) {
      throw std::runtime_error("lever_arm_frd must have exactly 3 elements");
    }
    lever_frd_ = Eigen::Vector3d(lever[0], lever[1], lever[2]);
    latency_s_ = this->get_parameter("latency_s").as_double();
    noise_stdev_ = this->get_parameter("position_noise_stdev").as_double();
    drift_stdev_ = this->get_parameter("drift_stdev_per_sqrt_m").as_double();
    rng_.seed(static_cast<unsigned>(this->get_parameter("seed").as_int()));

    flu_to_frd_ = Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();

    truth_sub_ = this->create_subscription<rosflight_msgs::msg::SimState>(
      "sim/truth_state", 10, [this](const rosflight_msgs::msg::SimState & msg) { truth_ = msg; have_truth_ = true; });
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 10);

    const double period = 1.0 / this->get_parameter("rate_hz").as_double();
    scan_timer_ = this->create_wall_timer(std::chrono::duration<double>(period),
                                          std::bind(&MockLioOdometry::sample_scan, this));
    // Releases queued estimates once their latency has elapsed; 2 ms keeps the added jitter small.
    release_timer_ = this->create_wall_timer(std::chrono::milliseconds(2),
                                             std::bind(&MockLioOdometry::release, this));
  }

private:
  rclcpp::Subscription<rosflight_msgs::msg::SimState>::SharedPtr truth_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
  rclcpp::TimerBase::SharedPtr release_timer_;

  rosflight_msgs::msg::SimState truth_;
  bool have_truth_ = false;

  Eigen::Vector3d lever_frd_;
  double latency_s_;
  double noise_stdev_;
  double drift_stdev_;
  Eigen::Matrix3d flu_to_frd_;

  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};

  bool initialized_ = false;
  Eigen::Matrix3d R_ned_init_;     // camera_init (FLU axes) -> NED
  Eigen::Vector3d p_init_ned_;     // NED position of the camera_init origin
  Eigen::Vector3d last_lidar_ned_;
  Eigen::Vector3d drift_ = Eigen::Vector3d::Zero();

  std::deque<std::pair<rclcpp::Time, nav_msgs::msg::Odometry>> pending_;

  Eigen::Vector3d gaussian3(double stdev)
  {
    return Eigen::Vector3d(normal_(rng_), normal_(rng_), normal_(rng_)) * stdev;
  }

  void sample_scan()
  {
    if (!have_truth_) {
      return;
    }

    const Eigen::Vector3d p_cg(truth_.pose.position.x, truth_.pose.position.y, truth_.pose.position.z);
    const Eigen::Matrix3d R_ned_body =
      Eigen::Quaterniond(truth_.pose.orientation.w, truth_.pose.orientation.x,
                         truth_.pose.orientation.y, truth_.pose.orientation.z)
        .normalized().toRotationMatrix();
    const Eigen::Vector3d p_lidar = p_cg + R_ned_body * lever_frd_;

    // camera_init is the LiDAR IMU frame (FLU) wherever it was when FAST-LIO started.
    if (!initialized_) {
      R_ned_init_ = R_ned_body * flu_to_frd_;
      p_init_ned_ = p_lidar;
      last_lidar_ned_ = p_lidar;
      initialized_ = true;
    }

    const double moved = (p_lidar - last_lidar_ned_).norm();
    last_lidar_ned_ = p_lidar;
    drift_ += gaussian3(drift_stdev_ * std::sqrt(moved));

    const Eigen::Vector3d p_flu =
      R_ned_init_.transpose() * (p_lidar - p_init_ned_) + drift_ + gaussian3(noise_stdev_);
    const Eigen::Quaterniond q_flu(R_ned_init_.transpose() * R_ned_body * flu_to_frd_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = truth_.header.stamp;
    odom.header.frame_id = "camera_init";
    odom.child_frame_id = "body";
    odom.pose.pose.position.x = p_flu.x();
    odom.pose.pose.position.y = p_flu.y();
    odom.pose.pose.position.z = p_flu.z();
    odom.pose.pose.orientation.w = q_flu.w();
    odom.pose.pose.orientation.x = q_flu.x();
    odom.pose.pose.orientation.y = q_flu.y();
    odom.pose.pose.orientation.z = q_flu.z();

    pending_.emplace_back(this->now() + rclcpp::Duration::from_seconds(latency_s_), odom);
  }

  void release()
  {
    const rclcpp::Time now = this->now();
    while (!pending_.empty() && pending_.front().first <= now) {
      odom_pub_->publish(pending_.front().second);
      pending_.pop_front();
    }
  }
};

} // namespace roscopter_sim

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roscopter_sim::MockLioOdometry>());
  rclcpp::shutdown();
  return 0;
}
