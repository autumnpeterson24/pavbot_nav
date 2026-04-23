#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class GuidanceCmdVelFollower : public rclcpp::Node
{
public:
  GuidanceCmdVelFollower()
  : Node("guidance_cmd_vel_follower")
  {
    // Topics
    this->declare_parameter<std::string>("path_topic", "/lanes/centerline");
    this->declare_parameter<std::string>("confidence_topic", "/lanes/confidence");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    // Timing / gating
    this->declare_parameter<double>("control_hz", 20.0);
    this->declare_parameter<double>("path_stale_sec", 0.30);
    this->declare_parameter<int>("min_path_points", 5);
    this->declare_parameter<double>("stop_if_confidence_below", 0.20);

    // Lookahead / point picking
    this->declare_parameter<double>("lookahead_m", 4.0);
    this->declare_parameter<double>("min_forward_x_m", 0.35);

    // Control gains
    this->declare_parameter<double>("k_linear", 0.35);
    this->declare_parameter<double>("k_angular", 1.80);

    // Limits
    this->declare_parameter<double>("max_linear_mps", 0.30);
    this->declare_parameter<double>("max_angular_rps", 0.50);
    this->declare_parameter<double>("slowdown_yaw_error_deg", 20.0);

    // Stop behavior
    this->declare_parameter<bool>("publish_zero_when_invalid", true);

    path_topic_ = this->get_parameter("path_topic").as_string();
    confidence_topic_ = this->get_parameter("confidence_topic").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();

    control_hz_ = this->get_parameter("control_hz").as_double();
    path_stale_sec_ = this->get_parameter("path_stale_sec").as_double();
    min_path_points_ = this->get_parameter("min_path_points").as_int();
    stop_if_confidence_below_ = this->get_parameter("stop_if_confidence_below").as_double();

    lookahead_m_ = this->get_parameter("lookahead_m").as_double();
    min_forward_x_m_ = this->get_parameter("min_forward_x_m").as_double();

    k_linear_ = this->get_parameter("k_linear").as_double();
    k_angular_ = this->get_parameter("k_angular").as_double();

    max_linear_mps_ = this->get_parameter("max_linear_mps").as_double();
    max_angular_rps_ = this->get_parameter("max_angular_rps").as_double();
    slowdown_yaw_error_deg_ = this->get_parameter("slowdown_yaw_error_deg").as_double();

    publish_zero_when_invalid_ = this->get_parameter("publish_zero_when_invalid").as_bool();

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10),
      std::bind(&GuidanceCmdVelFollower::onPath, this, std::placeholders::_1));

    conf_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      confidence_topic_, rclcpp::QoS(10),
      std::bind(&GuidanceCmdVelFollower::onConfidence, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(10));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&GuidanceCmdVelFollower::tick, this));

    RCLCPP_INFO(this->get_logger(),
      "guidance_cmd_vel_follower ready. path='%s' conf='%s' -> cmd='%s'",
      path_topic_.c_str(), confidence_topic_.c_str(), cmd_vel_topic_.c_str());
  }

private:
  static double clampd(double v, double lo, double hi)
  {
    return std::max(lo, std::min(hi, v));
  }

  static double wrapPi(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    last_path_ = *msg;
    last_path_time_ = this->now();
  }

  void onConfidence(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    last_confidence_ = msg->data;
    last_conf_time_ = this->now();
  }

  std::optional<geometry_msgs::msg::PoseStamped> pickLookaheadPose(
    const nav_msgs::msg::Path & path, double lookahead_m) const
  {
    if (path.poses.empty()) {
      return std::nullopt;
    }

    if (path.poses.size() == 1) {
      return path.poses.front();
    }

    double acc = 0.0;
    size_t chosen = path.poses.size() - 1;

    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & p0 = path.poses[i - 1].pose.position;
      const auto & p1 = path.poses[i].pose.position;
      acc += std::hypot(p1.x - p0.x, p1.y - p0.y);
      if (acc >= lookahead_m) {
        chosen = i;
        break;
      }
    }

    return path.poses[chosen];
  }

  void publishZero()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  void tick()
  {
    nav_msgs::msg::Path path;
    rclcpp::Time path_time;
    float conf = 0.0f;
    rclcpp::Time conf_time;

    {
      std::lock_guard<std::mutex> lk(mutex_);
      path = last_path_;
      path_time = last_path_time_;
      conf = last_confidence_;
      conf_time = last_conf_time_;
    }

    if (path.poses.size() < static_cast<size_t>(min_path_points_)) {
      if (publish_zero_when_invalid_) publishZero();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Path too short or not received yet.");
      return;
    }

    const double age = (this->now() - path_time).seconds();
    if (age > path_stale_sec_) {
      if (publish_zero_when_invalid_) publishZero();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Path stale: age=%.3f sec", age);
      return;
    }

    if (conf_time.nanoseconds() != 0 && conf < stop_if_confidence_below_) {
      if (publish_zero_when_invalid_) publishZero();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Lane confidence too low: %.3f", conf);
      return;
    }

    auto maybe_goal = pickLookaheadPose(path, lookahead_m_);
    if (!maybe_goal) {
      if (publish_zero_when_invalid_) publishZero();
      return;
    }

    // Path is already in base_link for your current lane detector.
    const double x = maybe_goal->pose.position.x;
    const double y = maybe_goal->pose.position.y;

    if (!std::isfinite(x) || !std::isfinite(y) || x < min_forward_x_m_) {
      if (publish_zero_when_invalid_) publishZero();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Lookahead point invalid or too close: x=%.3f y=%.3f", x, y);
      return;
    }

    const double yaw_error = wrapPi(std::atan2(y, x));
    const double yaw_error_abs = std::abs(yaw_error);
    const double slowdown_thresh = slowdown_yaw_error_deg_ * M_PI / 180.0;

    // Base commands
    double linear_x = k_linear_ * x;
    double angular_z = k_angular_ * yaw_error;

    // Slow down on big heading error so it doesn't drive hard while misaligned
    if (yaw_error_abs > slowdown_thresh) {
      const double factor = clampd(slowdown_thresh / yaw_error_abs, 0.2, 1.0);
      linear_x *= factor;
    }

    linear_x = clampd(linear_x, 0.0, max_linear_mps_);
    angular_z = clampd(angular_z, -max_angular_rps_, max_angular_rps_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.angular.z = angular_z;
    cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "cmd_vel: v=%.3f w=%.3f | lookahead=(%.3f, %.3f) conf=%.3f",
      linear_x, angular_z, x, y, conf);
  }

  // Params
  std::string path_topic_;
  std::string confidence_topic_;
  std::string cmd_vel_topic_;

  double control_hz_{20.0};
  double path_stale_sec_{0.30};
  int min_path_points_{5};
  double stop_if_confidence_below_{0.20};

  double lookahead_m_{4.0};
  double min_forward_x_m_{0.35};

  double k_linear_{0.35};
  double k_angular_{1.80};

  double max_linear_mps_{0.30};
  double max_angular_rps_{0.50};
  double slowdown_yaw_error_deg_{20.0};

  bool publish_zero_when_invalid_{true};

  // State
  std::mutex mutex_;
  nav_msgs::msg::Path last_path_;
  rclcpp::Time last_path_time_{0, 0, RCL_ROS_TIME};

  float last_confidence_{0.0f};
  rclcpp::Time last_conf_time_{0, 0, RCL_ROS_TIME};

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr conf_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuidanceCmdVelFollower>());
  rclcpp::shutdown();
  return 0;
}