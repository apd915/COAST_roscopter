#include "navigation/trajectory_follower_ros.hpp"
#include "navigation/trajectory_follower.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace roscopter
{

TrajectoryFollowerROS::TrajectoryFollowerROS()
  : Node("trajectory_follower")
  , params(this)
  , xhat_(roscopter_msgs::msg::State())
  , firmware_status_(rosflight_msgs::msg::Status())
  , received_cmd_msg_(false)
  , input_cmd_(roscopter_msgs::msg::TrajectoryCommand())
{
  // Instantiate publishers and subscribers
  state_sub_ = this->create_subscription<roscopter_msgs::msg::State>("estimated_state", 1, std::bind(&TrajectoryFollowerROS::state_callback, this, _1));

  // Sim-only. Absent on hardware, in which case has_truth stays false and the truth fields
  // are left at zero rather than silently reporting the estimate as if it were truth.
  truth_sub_ = this->create_subscription<rosflight_msgs::msg::SimState>(
    "sim/truth_state", 1, [this](const rosflight_msgs::msg::SimState::SharedPtr msg) {
      truth_ = *msg;
      have_truth_ = true;
    });
  trajectory_sub_ = this->create_subscription<roscopter_msgs::msg::TrajectoryCommand>("trajectory_command", 1, std::bind(&TrajectoryFollowerROS::cmd_callback, this, _1));
  firmware_status_sub_ = this->create_subscription<rosflight_msgs::msg::Status>("status", 1, std::bind(&TrajectoryFollowerROS::status_callback, this, _1));
  cmd_pub_ = this->create_publisher<roscopter_msgs::msg::ControllerCommand>("high_level_command", 1);
  tracking_error_pub_ = this->create_publisher<roscopter_msgs::msg::TrackingError>("tracking_error", 1);

  // Clear integrators service
  clear_integrators_srvs_ = this->create_service<std_srvs::srv::Trigger>(
    "trajectory_follower/clear_integrators", std::bind(&TrajectoryFollowerROS::clear_integrators_callback, this, _1, _2));

  // Register parameter callback
  parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&TrajectoryFollowerROS::parameters_callback, this, _1));
  
  declare_params();
  params.set_parameters();
}

void TrajectoryFollowerROS::declare_params()
{
  // Declare params here needed in this node
}

rcl_interfaces::msg::SetParametersResult TrajectoryFollowerROS::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "One of the parameters is not a parameter of the trajectory follower.";

  // Use the ParamManager's set parameters callback
  bool success = params.set_parameters_callback(parameters);
  if (success)
  {
    result.successful = true;
    result.reason = "success";
  }

  // Call the update_gains method implemented in a derived class to save the gains in member variables there.
  update_gains();

  return result;
}

void TrajectoryFollowerROS::state_callback(const roscopter_msgs::msg::State &msg)
{
  xhat_ = msg;

  // If the trajectory follower has not received an input command yet, do not compute control commands or publish
  if (!received_cmd_msg_) { return; }

  // Calculate dt and return if dt is invalid
  double now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
  double dt = compute_dt(now);

  roscopter_msgs::msg::ControllerCommand output_cmd = manage_trajectory(input_cmd_, dt);

  publish_command(output_cmd);
  publish_tracking_error();
}

double TrajectoryFollowerROS::compute_dt(double now)
{
  static double prev_time = 0;
  if(prev_time == 0) {
    prev_time = now;
    // Don't compute control since we don't have a dt calculation
    return 0;
  }

  // Calculate time
  double dt = now - prev_time;
  prev_time = now;

  return dt;
}

void TrajectoryFollowerROS::cmd_callback(const roscopter_msgs::msg::TrajectoryCommand &msg)
{
  input_cmd_ = msg;
  received_cmd_msg_ = true;
}

void TrajectoryFollowerROS::status_callback(const rosflight_msgs::msg::Status &msg)
{
  firmware_status_ = msg;
}

void TrajectoryFollowerROS::publish_tracking_error()
{
  roscopter_msgs::msg::TrackingError msg;
  msg.header.stamp = this->get_clock()->now();

  // Commanded minus actual. xhat_ is NED, and TrajectoryCommand.position is NED, so these
  // are directly comparable with no frame conversion.
  const double en = input_cmd_.position[0] - xhat_.p_n;
  const double ee = input_cmd_.position[1] - xhat_.p_e;
  const double ed = input_cmd_.position[2] - xhat_.p_d;
  msg.position_error = {en, ee, ed};
  msg.position_error_norm = std::sqrt(en * en + ee * ee + ed * ed);

  // xhat_ velocities are BODY frame; rotate to inertial so the comparison against the
  // (inertial) commanded velocity is meaningful.
  Eigen::Quaterniond q_body_to_inertial(xhat_.quat.w, xhat_.quat.x, xhat_.quat.y, xhat_.quat.z);
  Eigen::Vector3d v_inertial = q_body_to_inertial * Eigen::Vector3d(xhat_.v_x, xhat_.v_y, xhat_.v_z);
  const double vn = input_cmd_.velocity[0] - v_inertial.x();
  const double ve = input_cmd_.velocity[1] - v_inertial.y();
  const double vd = input_cmd_.velocity[2] - v_inertial.z();
  msg.velocity_error = {vn, ve, vd};
  msg.velocity_error_norm = std::sqrt(vn * vn + ve * ve + vd * vd);

  // The clearance the planner inflates its corridors by. Kept as a parameter rather than
  // hard-coded so it can be matched to the planner's drone_physical_radius; if the two ever
  // disagree, margin_exceeded is measuring against the wrong thing.
  msg.safety_margin = params.get_double("safety_margin");
  msg.margin_exceeded = msg.position_error_norm > msg.safety_margin;

  peak_position_error_ = std::max(peak_position_error_, msg.position_error_norm);
  msg.peak_position_error = peak_position_error_;

  // Separate CONTROL error from ESTIMATOR error. Everything above is measured against
  // estimated_state -- the very signal the controller regulates -- so a biased estimate makes
  // position_error read small while the aircraft is visibly off the trajectory. Tightening
  // gains in that case only tracks the wrong number harder.
  msg.has_truth = have_truth_;
  if (have_truth_) {
    const double tn = input_cmd_.position[0] - truth_.pose.position.x;
    const double te = input_cmd_.position[1] - truth_.pose.position.y;
    const double td = input_cmd_.position[2] - truth_.pose.position.z;
    msg.true_position_error = {tn, te, td};
    msg.true_position_error_norm = std::sqrt(tn * tn + te * te + td * td);

    const double bn = truth_.pose.position.x - xhat_.p_n;
    const double be = truth_.pose.position.y - xhat_.p_e;
    const double bd = truth_.pose.position.z - xhat_.p_d;
    msg.estimator_error = {bn, be, bd};
    msg.estimator_error_norm = std::sqrt(bn * bn + be * be + bd * bd);

    // The guarantee is about where the vehicle IS, not where it thinks it is.
    msg.margin_exceeded = msg.true_position_error_norm > msg.safety_margin;
    peak_position_error_ = std::max(peak_position_error_, msg.true_position_error_norm);
    msg.peak_position_error = peak_position_error_;
  }

  tracking_error_pub_->publish(msg);
}

void TrajectoryFollowerROS::publish_command(roscopter_msgs::msg::ControllerCommand &command)
{
  command.header.stamp = this->get_clock()->now();
  cmd_pub_->publish(command);
}

bool TrajectoryFollowerROS::clear_integrators_callback(const std_srvs::srv::Trigger::Request::SharedPtr req,
                                                       const std_srvs::srv::Trigger::Response::SharedPtr res)
{
  clear_integrators();
  res->success = true;
  return true;
}

double TrajectoryFollowerROS::saturate(double x, double max, double min)
{
  x = (x > max) ? max : x;
  x = (x < min) ? min : x;
  return x;
}

}   // namespace roscopter


int main(int argc, char* argv[])
{
    // Initialize the ROS2 client library
    rclcpp::init(argc, argv);

    // Create and spin node
    auto node = std::make_shared<roscopter::TrajectoryFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
