/**
 * Turns raw LiDAR scans into what minimum_snap_manager::lidar_cb consumes: hit points in world NED.
 *
 * Input is the sensor's own frame, as livox_ros_driver2 publishes it (PointCloud2, x forward,
 * y left, z up, metres from the sensor). Per scan:
 *
 *   1. Drop invalid points: non-finite, inside min_range (which is also where the driver's
 *      (0,0,0) no-return points land), or beyond max_range when one is set. A (0,0,0) that
 *      survived would be stamped OCCUPIED at the vehicle's own position.
 *   2. Drop self-hits: anything inside a body-frame box around the airframe (props, arms).
 *   3. Place the rest in the world: sensor FLU -> body FRD (180 deg roll, plus the mounting
 *      offset) -> NED, using the ESTIMATED pose nearest the scan's stamp. Using the estimate,
 *      not truth, is the point: the map then carries the estimator's error exactly as it will
 *      on the vehicle.
 *   4. Optionally keep one point per downsample_voxel cell, in NED, so a full-rate Mid-360
 *      scan (~20k points) costs the planner's raycaster roughly what the old 3-degree grid did.
 *
 * Steps 2 and 4 default OFF so that enabling this node on its own changes only the frame and the
 * pose source; each is switched on in the step that introduces what it handles.
 */
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <roscopter_msgs/msg/state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace roscopter
{

class LidarPreprocessor : public rclcpp::Node
{
public:
  LidarPreprocessor()
      : Node("lidar_preprocessor")
  {
    this->declare_parameter("min_range", 0.1);       // m; Mid-360 blind zone
    this->declare_parameter("max_range", 0.0);       // m; 0 = keep every return
    // Self-filter box in body FRD, metres. Equal corners = disabled.
    this->declare_parameter("self_filter_min_frd", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter("self_filter_max_frd", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter("downsample_voxel", 0.0);  // m; 0 = disabled
    // Sensor origin relative to the CG, body FRD, metres.
    this->declare_parameter("lever_arm_frd", std::vector<double>{0.0, 0.0, 0.0});
    // Scans with no pose estimate within this of their stamp are dropped, not guessed at.
    this->declare_parameter("max_pose_age_s", 0.05);

    min_range_ = this->get_parameter("min_range").as_double();
    max_range_ = this->get_parameter("max_range").as_double();
    downsample_voxel_ = this->get_parameter("downsample_voxel").as_double();
    max_pose_age_s_ = this->get_parameter("max_pose_age_s").as_double();
    lever_frd_ = vector3_param("lever_arm_frd");
    self_min_ = vector3_param("self_filter_min_frd");
    self_max_ = vector3_param("self_filter_max_frd");
    self_filter_enabled_ = (self_max_ - self_min_).minCoeff() > 0.0;

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar/points_ned", 10);
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10, std::bind(&LidarPreprocessor::cloud_callback, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<roscopter_msgs::msg::State>(
      "estimated_state", 50, std::bind(&LidarPreprocessor::state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Range [%.2f, %s] m, self-filter %s, downsample %s, lever arm [%.3f, %.3f, %.3f] m FRD.",
                min_range_, max_range_ > 0.0 ? std::to_string(max_range_).c_str() : "inf",
                self_filter_enabled_ ? "ON" : "off",
                downsample_voxel_ > 0.0 ? std::to_string(downsample_voxel_).c_str() : "off",
                lever_frd_.x(), lever_frd_.y(), lever_frd_.z());
  }

private:
  struct Pose
  {
    rclcpp::Time stamp;
    Eigen::Vector3d p_ned;
    Eigen::Matrix3d R_ned_body;
  };

  struct VoxelHash
  {
    std::size_t operator()(const Eigen::Vector3i & v) const
    {
      // Large odd multipliers; libstdc++'s std::hash<int> is the identity, so XOR-ing raw
      // indices collides constantly on grid keys (see Vector3iHash in the planner).
      return static_cast<std::size_t>(v.x()) * 73856093u ^
             static_cast<std::size_t>(v.y()) * 19349663u ^
             static_cast<std::size_t>(v.z()) * 83492791u;
    }
  };

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<roscopter_msgs::msg::State>::SharedPtr state_sub_;

  double min_range_;
  double max_range_;
  double downsample_voxel_;
  double max_pose_age_s_;
  Eigen::Vector3d lever_frd_;
  Eigen::Vector3d self_min_;
  Eigen::Vector3d self_max_;
  bool self_filter_enabled_;

  std::deque<Pose> poses_;

  // Per-reporting-window counters, so a filter that is silently eating the scan is visible.
  std::size_t n_scans_ = 0, n_in_ = 0, n_out_ = 0;
  std::size_t n_invalid_ = 0, n_range_ = 0, n_self_ = 0, n_no_pose_ = 0;
  double ms_sum_ = 0.0, pose_dt_max_ = 0.0;

  Eigen::Vector3d vector3_param(const std::string & name)
  {
    const auto v = this->get_parameter(name).as_double_array();
    if (v.size() != 3) {
      throw std::runtime_error(name + " must have exactly 3 elements");
    }
    return Eigen::Vector3d(v[0], v[1], v[2]);
  }

  void state_callback(const roscopter_msgs::msg::State & msg)
  {
    Pose pose;
    pose.stamp = msg.header.stamp;
    pose.p_ned = Eigen::Vector3d(msg.p_n, msg.p_e, msg.p_d);
    pose.R_ned_body =
      Eigen::Quaterniond(msg.quat.w, msg.quat.x, msg.quat.y, msg.quat.z).normalized().toRotationMatrix();
    poses_.push_back(pose);
    while (poses_.size() > 400) {  // ~1 s at the estimator's 390 Hz
      poses_.pop_front();
    }
  }

  const Pose * nearest_pose(const rclcpp::Time & stamp, double & dt_out) const
  {
    const Pose * best = nullptr;
    dt_out = std::numeric_limits<double>::infinity();
    for (const auto & pose : poses_) {
      const double dt = std::abs((pose.stamp - stamp).seconds());
      if (dt < dt_out) {
        dt_out = dt;
        best = &pose;
      }
    }
    return best;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2 & msg)
  {
    const auto t0 = std::chrono::steady_clock::now();
    const std::size_t n_points = static_cast<std::size_t>(msg.width) * msg.height;

    double pose_dt;
    const Pose * pose = nearest_pose(msg.header.stamp, pose_dt);
    if (pose == nullptr || pose_dt > max_pose_age_s_) {
      ++n_no_pose_;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Dropping scan: nearest pose estimate is %.3f s from its stamp (limit %.3f s).",
                           pose_dt, max_pose_age_s_);
      return;
    }
    pose_dt_max_ = std::max(pose_dt_max_, pose_dt);

    const Eigen::Matrix3d & R = pose->R_ned_body;
    const Eigen::Vector3d p_sensor_ned = pose->p_ned + R * lever_frd_;
    const double min_sq = min_range_ * min_range_;
    const double max_sq = max_range_ * max_range_;

    std::vector<Eigen::Vector3f> out;
    out.reserve(n_points);
    std::unordered_set<Eigen::Vector3i, VoxelHash> occupied_voxels;

    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const Eigen::Vector3d p_flu(*it_x, *it_y, *it_z);
      if (!p_flu.allFinite()) {
        ++n_invalid_;
        continue;
      }
      const double r_sq = p_flu.squaredNorm();
      if (r_sq < min_sq || (max_range_ > 0.0 && r_sq > max_sq)) {
        ++n_range_;
        continue;
      }

      // FLU -> FRD is a 180 deg roll about x, then shift from the sensor to the CG.
      const Eigen::Vector3d p_body(p_flu.x(), -p_flu.y(), -p_flu.z());
      if (self_filter_enabled_) {
        const Eigen::Vector3d p_cg = p_body + lever_frd_;
        if ((p_cg.array() >= self_min_.array()).all() && (p_cg.array() <= self_max_.array()).all()) {
          ++n_self_;
          continue;
        }
      }

      const Eigen::Vector3d p_ned = p_sensor_ned + R * p_body;
      if (downsample_voxel_ > 0.0) {
        const Eigen::Vector3i key = (p_ned / downsample_voxel_).array().floor().cast<int>();
        if (!occupied_voxels.insert(key).second) {
          continue;
        }
      }
      out.push_back(p_ned.cast<float>());
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = msg.header.stamp;
    cloud.header.frame_id = "NED";
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(out.size());
    cloud.is_dense = true;
    cloud.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(out.size());
    sensor_msgs::PointCloud2Iterator<float> o_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> o_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> o_z(cloud, "z");
    for (const auto & p : out) {
      *o_x = p.x(); *o_y = p.y(); *o_z = p.z();
      ++o_x; ++o_y; ++o_z;
    }
    cloud_pub_->publish(cloud);

    n_in_ += n_points;
    n_out_ += out.size();
    ms_sum_ += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (++n_scans_ >= 50) {
      report();
    }
  }

  void report()
  {
    RCLCPP_INFO(this->get_logger(),
                "%zu scans: %.0f -> %.0f pts/scan (dropped: %zu invalid, %zu range, %zu self; "
                "%zu scans without a pose). %.2f ms/scan, pose up to %.1f ms from scan stamp.",
                n_scans_, static_cast<double>(n_in_) / n_scans_, static_cast<double>(n_out_) / n_scans_,
                n_invalid_, n_range_, n_self_, n_no_pose_, ms_sum_ / n_scans_, pose_dt_max_ * 1e3);
    n_scans_ = n_in_ = n_out_ = n_invalid_ = n_range_ = n_self_ = n_no_pose_ = 0;
    ms_sum_ = pose_dt_max_ = 0.0;
  }
};

} // namespace roscopter

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roscopter::LidarPreprocessor>());
  rclcpp::shutdown();
  return 0;
}
