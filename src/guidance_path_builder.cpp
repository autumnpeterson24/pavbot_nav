#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

struct Pt2
{
  double x{0.0};
  double y{0.0};
};

static inline double clampd(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

static inline double wrap_pi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static inline double dist2d(const Pt2 & a, const Pt2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

static inline double norm2d(const Pt2 & p)
{
  return std::hypot(p.x, p.y);
}

static inline Pt2 normalize2d(const Pt2 & p)
{
  const double n = norm2d(p);
  if (n < 1e-9) return Pt2{0.0, 0.0};
  return Pt2{p.x / n, p.y / n};
}

static inline Pt2 leftNormalFromYaw(double yaw)
{
  return Pt2{-std::sin(yaw), std::cos(yaw)};
}

static inline double yawFromPts(const Pt2 & a, const Pt2 & b)
{
  return std::atan2(b.y - a.y, b.x - a.x);
}

class GuidancePathBuilder : public rclcpp::Node
{
public:
  GuidancePathBuilder()
  : Node("guidance_path_builder")
  {
    // Frames / topics
    this->declare_parameter<std::string>("robot_frame", "base_link");
    this->declare_parameter<std::string>("guidance_topic", "/guidance/path");
    this->declare_parameter<std::string>("mode_topic", "/guidance/mode");

    this->declare_parameter<std::string>("centerline_topic", "/lanes/centerline");
    this->declare_parameter<std::string>("left_boundary_topic", "/lanes/left_boundary");
    this->declare_parameter<std::string>("right_boundary_topic", "/lanes/right_boundary");
    this->declare_parameter<std::string>("lane_confidence_topic", "/lanes/confidence");
    this->declare_parameter<std::string>("scan_topic", "/scan");

    // General behavior
    this->declare_parameter<double>("publish_hz", 10.0);
    this->declare_parameter<double>("path_stale_sec", 0.40);
    this->declare_parameter<int>("min_centerline_points", 5);
    this->declare_parameter<int>("min_boundary_points", 5);

    // Lane fallback behavior
    this->declare_parameter<double>("lane_confidence_threshold", 0.45);
    this->declare_parameter<double>("nominal_half_width_m", 2.20);
    this->declare_parameter<bool>("prefer_centerline_when_available", true);

    // Scan fallback path
    this->declare_parameter<double>("fallback_path_length_m", 8.0);
    this->declare_parameter<double>("fallback_point_spacing_m", 0.5);
    this->declare_parameter<double>("fallback_max_heading_deg", 70.0);
    this->declare_parameter<double>("fallback_heading_step_deg", 5.0);
    this->declare_parameter<double>("fallback_clearance_margin_m", 0.9);
    this->declare_parameter<double>("fallback_min_clear_range_m", 1.5);
    this->declare_parameter<double>("fallback_scan_cap_m", 12.0);
    this->declare_parameter<double>("fallback_heading_smooth_alpha", 0.85);

    // Optional path smoothing
    this->declare_parameter<bool>("smooth_output_path", true);
    this->declare_parameter<double>("path_smooth_alpha", 0.35);

    // NEW: turn-aware inside bias
    this->declare_parameter<bool>("enable_turn_inside_bias", true);
    this->declare_parameter<double>("turn_detect_lookahead_idx", 6.0);
    this->declare_parameter<double>("turn_detect_yaw_thresh_deg", 8.0);
    this->declare_parameter<double>("inside_bias_fraction", 0.14);
    this->declare_parameter<double>("inside_bias_max_m", 0.30);
    this->declare_parameter<double>("inside_bias_fade_length_m", 4.5);

    robot_frame_ = this->get_parameter("robot_frame").as_string();
    guidance_topic_ = this->get_parameter("guidance_topic").as_string();
    mode_topic_ = this->get_parameter("mode_topic").as_string();

    centerline_topic_ = this->get_parameter("centerline_topic").as_string();
    left_boundary_topic_ = this->get_parameter("left_boundary_topic").as_string();
    right_boundary_topic_ = this->get_parameter("right_boundary_topic").as_string();
    lane_conf_topic_ = this->get_parameter("lane_confidence_topic").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();

    publish_hz_ = this->get_parameter("publish_hz").as_double();
    path_stale_sec_ = this->get_parameter("path_stale_sec").as_double();
    min_centerline_points_ = this->get_parameter("min_centerline_points").as_int();
    min_boundary_points_ = this->get_parameter("min_boundary_points").as_int();

    lane_conf_thresh_ = this->get_parameter("lane_confidence_threshold").as_double();
    nominal_half_width_m_ = this->get_parameter("nominal_half_width_m").as_double();
    prefer_centerline_when_available_ =
      this->get_parameter("prefer_centerline_when_available").as_bool();

    fallback_path_length_m_ = this->get_parameter("fallback_path_length_m").as_double();
    fallback_point_spacing_m_ = this->get_parameter("fallback_point_spacing_m").as_double();
    fallback_max_heading_deg_ = this->get_parameter("fallback_max_heading_deg").as_double();
    fallback_heading_step_deg_ = this->get_parameter("fallback_heading_step_deg").as_double();
    fallback_clearance_margin_m_ = this->get_parameter("fallback_clearance_margin_m").as_double();
    fallback_min_clear_range_m_ = this->get_parameter("fallback_min_clear_range_m").as_double();
    fallback_scan_cap_m_ = this->get_parameter("fallback_scan_cap_m").as_double();
    fallback_heading_smooth_alpha_ =
      this->get_parameter("fallback_heading_smooth_alpha").as_double();

    smooth_output_path_ = this->get_parameter("smooth_output_path").as_bool();
    path_smooth_alpha_ = this->get_parameter("path_smooth_alpha").as_double();

    enable_turn_inside_bias_ =
      this->get_parameter("enable_turn_inside_bias").as_bool();
    turn_detect_lookahead_idx_ =
      this->get_parameter("turn_detect_lookahead_idx").as_double();
    turn_detect_yaw_thresh_deg_ =
      this->get_parameter("turn_detect_yaw_thresh_deg").as_double();
    inside_bias_fraction_ =
      this->get_parameter("inside_bias_fraction").as_double();
    inside_bias_max_m_ =
      this->get_parameter("inside_bias_max_m").as_double();
    inside_bias_fade_length_m_ =
      this->get_parameter("inside_bias_fade_length_m").as_double();

    // Subscribers
    centerline_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      centerline_topic_, rclcpp::QoS(10),
      std::bind(&GuidancePathBuilder::onCenterline, this, std::placeholders::_1));

    left_boundary_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      left_boundary_topic_, rclcpp::QoS(10),
      std::bind(&GuidancePathBuilder::onLeftBoundary, this, std::placeholders::_1));

    right_boundary_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      right_boundary_topic_, rclcpp::QoS(10),
      std::bind(&GuidancePathBuilder::onRightBoundary, this, std::placeholders::_1));

    lane_conf_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      lane_conf_topic_, rclcpp::QoS(10),
      std::bind(&GuidancePathBuilder::onLaneConfidence, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::QoS(10),
      std::bind(&GuidancePathBuilder::onScan, this, std::placeholders::_1));

    // Publishers
    guidance_pub_ = this->create_publisher<nav_msgs::msg::Path>(guidance_topic_, rclcpp::QoS(10));
    mode_pub_ = this->create_publisher<std_msgs::msg::String>(mode_topic_, rclcpp::QoS(10));

    // Timer
    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&GuidancePathBuilder::tick, this));

    RCLCPP_INFO(this->get_logger(),
      "guidance_path_builder ready. centerline='%s' left='%s' right='%s' scan='%s' -> guidance='%s'",
      centerline_topic_.c_str(),
      left_boundary_topic_.c_str(),
      right_boundary_topic_.c_str(),
      scan_topic_.c_str(),
      guidance_topic_.c_str());
  }

private:
  void onCenterline(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    centerline_ = *msg;
    centerline_time_ = this->now();
  }

  void onLeftBoundary(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    left_boundary_ = *msg;
    left_boundary_time_ = this->now();
  }

  void onRightBoundary(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    right_boundary_ = *msg;
    right_boundary_time_ = this->now();
  }

  void onLaneConfidence(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    lane_confidence_ = msg->data;
    lane_confidence_time_ = this->now();
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    last_scan_ = *msg;
    scan_time_ = this->now();
  }

  void tick()
  {
    nav_msgs::msg::Path centerline_copy;
    nav_msgs::msg::Path left_copy;
    nav_msgs::msg::Path right_copy;
    sensor_msgs::msg::LaserScan scan_copy;
    double lane_conf = 0.0;

    rclcpp::Time centerline_time;
    rclcpp::Time left_time;
    rclcpp::Time right_time;
    rclcpp::Time conf_time;
    rclcpp::Time scan_time;

    {
      std::lock_guard<std::mutex> lk(mutex_);
      centerline_copy = centerline_;
      left_copy = left_boundary_;
      right_copy = right_boundary_;
      scan_copy = last_scan_;
      lane_conf = lane_confidence_;

      centerline_time = centerline_time_;
      left_time = left_boundary_time_;
      right_time = right_boundary_time_;
      conf_time = lane_confidence_time_;
      scan_time = scan_time_;
    }

    const bool centerline_ok =
      isFresh(centerline_time) &&
      static_cast<int>(centerline_copy.poses.size()) >= min_centerline_points_;

    const bool left_ok =
      isFresh(left_time) &&
      static_cast<int>(left_copy.poses.size()) >= min_boundary_points_;

    const bool right_ok =
      isFresh(right_time) &&
      static_cast<int>(right_copy.poses.size()) >= min_boundary_points_;

    const bool conf_ok = isFresh(conf_time);
    const bool lane_conf_good = (!conf_ok) ? true : (lane_conf >= lane_conf_thresh_);

    nav_msgs::msg::Path out;
    std::string mode = "NONE";

    if (prefer_centerline_when_available_ && centerline_ok && lane_conf_good) {
      out = centerline_copy;
      if (smooth_output_path_) {
        smoothPathInPlace(out);
      } else {
        orientPathTangential(out);
      }
      applyTurnInsideBias(out);
      mode = "CENTERLINE";
    } else if (left_ok && right_ok && lane_conf_good) {
      out = midpointPath(left_copy, right_copy);
      if (smooth_output_path_) {
        smoothPathInPlace(out);
      } else {
        orientPathTangential(out);
      }
      applyTurnInsideBias(out);
      mode = "MIDPOINT_FROM_BOUNDARIES";
    } else if (left_ok && lane_conf_good) {
      out = offsetBoundaryPath(left_copy, -nominal_half_width_m_);
      if (smooth_output_path_) {
        smoothPathInPlace(out);
      } else {
        orientPathTangential(out);
      }
      applyTurnInsideBias(out);
      mode = "LEFT_ONLY";
    } else if (right_ok && lane_conf_good) {
      out = offsetBoundaryPath(right_copy, +nominal_half_width_m_);
      if (smooth_output_path_) {
        smoothPathInPlace(out);
      } else {
        orientPathTangential(out);
      }
      applyTurnInsideBias(out);
      mode = "RIGHT_ONLY";
    } else if (isFresh(scan_time) && !scan_copy.ranges.empty()) {
      auto maybe = buildScanFallbackPath(scan_copy);
      if (maybe) {
        out = *maybe;
        mode = "SCAN_FALLBACK";
      }
    }

    if (mode == "NONE" || out.poses.size() < 2) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "guidance_path_builder has no valid guidance mode yet.");
      return;
    }

    out.header.stamp = this->now();
    guidance_pub_->publish(out);

    std_msgs::msg::String mode_msg;
    mode_msg.data = mode;
    mode_pub_->publish(mode_msg);
  }

  bool isFresh(const rclcpp::Time & t) const
  {
    if (t.nanoseconds() == 0) return false;
    return (this->now() - t).seconds() <= path_stale_sec_;
  }

  static Pt2 getPt(const geometry_msgs::msg::PoseStamped & ps)
  {
    return Pt2{ps.pose.position.x, ps.pose.position.y};
  }

  static void setYaw(geometry_msgs::msg::PoseStamped & ps, double yaw)
  {
    ps.pose.orientation.x = 0.0;
    ps.pose.orientation.y = 0.0;
    ps.pose.orientation.z = std::sin(0.5 * yaw);
    ps.pose.orientation.w = std::cos(0.5 * yaw);
  }

  nav_msgs::msg::Path midpointPath(
    const nav_msgs::msg::Path & left,
    const nav_msgs::msg::Path & right) const
  {
    nav_msgs::msg::Path out;
    out.header = left.header;

    const size_t n = std::min(left.poses.size(), right.poses.size());
    out.poses.reserve(n);

    for (size_t i = 0; i < n; ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header = left.header;
      p.pose.position.x = 0.5 * (left.poses[i].pose.position.x + right.poses[i].pose.position.x);
      p.pose.position.y = 0.5 * (left.poses[i].pose.position.y + right.poses[i].pose.position.y);
      p.pose.position.z = 0.0;
      out.poses.push_back(p);
    }

    orientPathTangential(out);
    return out;
  }

  nav_msgs::msg::Path offsetBoundaryPath(
    const nav_msgs::msg::Path & in,
    double signed_offset_m) const
  {
    nav_msgs::msg::Path out;
    out.header = in.header;
    out.poses.resize(in.poses.size());

    if (in.poses.size() < 2) {
      return out;
    }

    for (size_t i = 0; i < in.poses.size(); ++i) {
      const Pt2 p = getPt(in.poses[i]);

      Pt2 a, b;
      if (i == 0) {
        a = getPt(in.poses[i]);
        b = getPt(in.poses[i + 1]);
      } else if (i + 1 >= in.poses.size()) {
        a = getPt(in.poses[i - 1]);
        b = getPt(in.poses[i]);
      } else {
        a = getPt(in.poses[i - 1]);
        b = getPt(in.poses[i + 1]);
      }

      const double yaw = yawFromPts(a, b);
      const Pt2 n = leftNormalFromYaw(yaw);

      out.poses[i].header = in.header;
      out.poses[i].pose.position.x = p.x + signed_offset_m * n.x;
      out.poses[i].pose.position.y = p.y + signed_offset_m * n.y;
      out.poses[i].pose.position.z = 0.0;
      setYaw(out.poses[i], yaw);
    }

    return out;
  }

  void orientPathTangential(nav_msgs::msg::Path & path) const
  {
    if (path.poses.size() < 2) return;

    for (size_t i = 0; i < path.poses.size(); ++i) {
      Pt2 a, b;
      if (i == 0) {
        a = getPt(path.poses[0]);
        b = getPt(path.poses[1]);
      } else if (i + 1 >= path.poses.size()) {
        a = getPt(path.poses[path.poses.size() - 2]);
        b = getPt(path.poses[path.poses.size() - 1]);
      } else {
        a = getPt(path.poses[i - 1]);
        b = getPt(path.poses[i + 1]);
      }
      const double yaw = yawFromPts(a, b);
      setYaw(path.poses[i], yaw);
    }
  }

  void smoothPathInPlace(nav_msgs::msg::Path & path) const
  {
    if (!smooth_output_path_ || path.poses.size() < 3) {
      orientPathTangential(path);
      return;
    }

    const double a = clampd(path_smooth_alpha_, 0.0, 0.95);

    std::vector<Pt2> pts(path.poses.size());
    for (size_t i = 0; i < path.poses.size(); ++i) {
      pts[i] = getPt(path.poses[i]);
    }

    std::vector<Pt2> sm = pts;

    for (size_t i = 1; i + 1 < pts.size(); ++i) {
      sm[i].x = (1.0 - a) * pts[i].x + 0.5 * a * (pts[i - 1].x + pts[i + 1].x);
      sm[i].y = (1.0 - a) * pts[i].y + 0.5 * a * (pts[i - 1].y + pts[i + 1].y);
    }

    for (size_t i = 0; i < path.poses.size(); ++i) {
      path.poses[i].pose.position.x = sm[i].x;
      path.poses[i].pose.position.y = sm[i].y;
      path.poses[i].pose.position.z = 0.0;
    }

    orientPathTangential(path);
  }

  double estimatePathTurnSigned(const nav_msgs::msg::Path & path) const
  {
    if (path.poses.size() < 4) {
      return 0.0;
    }

    const size_t i0 = 0;
    const size_t i1 = std::min<size_t>(2, path.poses.size() - 1);
    const size_t i2 = std::min<size_t>(
      static_cast<size_t>(std::round(turn_detect_lookahead_idx_)),
      path.poses.size() - 1);

    if (i2 <= i1) {
      return 0.0;
    }

    const Pt2 p0 = getPt(path.poses[i0]);
    const Pt2 p1 = getPt(path.poses[i1]);
    const Pt2 p2 = getPt(path.poses[i2]);

    const double yaw_near = yawFromPts(p0, p1);
    const double yaw_far = yawFromPts(p1, p2);

    return wrap_pi(yaw_far - yaw_near);
  }

  void applyTurnInsideBias(nav_msgs::msg::Path & path) const
  {
    if (!enable_turn_inside_bias_ || path.poses.size() < 3) {
      return;
    }

    const double signed_turn = estimatePathTurnSigned(path);
    const double yaw_thresh = turn_detect_yaw_thresh_deg_ * M_PI / 180.0;

    if (std::abs(signed_turn) < yaw_thresh) {
      return;
    }

    const double turn_strength =
      clampd(std::abs(signed_turn) / (25.0 * M_PI / 180.0), 0.0, 1.0);

    const double base_bias =
      std::min(inside_bias_max_m_, inside_bias_fraction_ * nominal_half_width_m_);

    const double signed_bias =
      (signed_turn > 0.0 ? +1.0 : -1.0) * base_bias * turn_strength;

    double accum_s = 0.0;

    for (size_t i = 0; i < path.poses.size(); ++i) {
      Pt2 a, b;
      if (i == 0) {
        a = getPt(path.poses[i]);
        b = getPt(path.poses[i + 1]);
      } else if (i + 1 >= path.poses.size()) {
        a = getPt(path.poses[i - 1]);
        b = getPt(path.poses[i]);
      } else {
        a = getPt(path.poses[i - 1]);
        b = getPt(path.poses[i + 1]);
      }

      if (i > 0) {
        accum_s += dist2d(getPt(path.poses[i - 1]), getPt(path.poses[i]));
      }

      const double yaw = yawFromPts(a, b);
      const Pt2 n = leftNormalFromYaw(yaw);

      const double fade =
        clampd(1.0 - (accum_s / std::max(0.1, inside_bias_fade_length_m_)), 0.0, 1.0);

      path.poses[i].pose.position.x += signed_bias * fade * n.x;
      path.poses[i].pose.position.y += signed_bias * fade * n.y;
    }

    orientPathTangential(path);
  }

  double getRangeAtAngle(const sensor_msgs::msg::LaserScan & scan, double angle_rad) const
  {
    if (scan.ranges.empty()) return 0.0;

    const double idx_f = (angle_rad - scan.angle_min) / scan.angle_increment;
    int idx = static_cast<int>(std::round(idx_f));
    idx = std::max(0, std::min(static_cast<int>(scan.ranges.size()) - 1, idx));

    double r = scan.ranges[idx];
    if (!std::isfinite(r)) {
      r = scan.range_max;
    }

    if (r <= 0.0) {
      r = scan.range_min;
    }

    r = clampd(r, scan.range_min, scan.range_max);
    r = std::min(r, fallback_scan_cap_m_);
    return r;
  }

  double scoreHeading(const sensor_msgs::msg::LaserScan & scan, double heading_rad) const
  {
    const double spread = 12.0 * M_PI / 180.0;

    const double r0 = getRangeAtAngle(scan, heading_rad);
    const double r1 = getRangeAtAngle(scan, heading_rad - spread);
    const double r2 = getRangeAtAngle(scan, heading_rad + spread);

    const double clearance = std::min({r0, r1, r2});
    const double steering_penalty = 0.25 * std::abs(heading_rad);

    return clearance - steering_penalty;
  }

  std::optional<nav_msgs::msg::Path> buildScanFallbackPath(
    const sensor_msgs::msg::LaserScan & scan)
  {
    const double max_h = fallback_max_heading_deg_ * M_PI / 180.0;
    const double step_h = std::max(1.0, fallback_heading_step_deg_) * M_PI / 180.0;

    double best_heading = 0.0;
    double best_score = -1e9;
    double best_clearance = 0.0;

    for (double h = -max_h; h <= max_h + 1e-9; h += step_h) {
      const double score = scoreHeading(scan, h);
      const double clearance = std::min({
        getRangeAtAngle(scan, h),
        getRangeAtAngle(scan, h - 10.0 * M_PI / 180.0),
        getRangeAtAngle(scan, h + 10.0 * M_PI / 180.0)
      });

      if (score > best_score) {
        best_score = score;
        best_heading = h;
        best_clearance = clearance;
      }
    }

    if (best_clearance < fallback_min_clear_range_m_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1500,
        "SCAN_FALLBACK blocked: best clearance %.2f m < min %.2f m",
        best_clearance, fallback_min_clear_range_m_);
      return std::nullopt;
    }

    if (!has_last_fallback_heading_) {
      last_fallback_heading_ = best_heading;
      has_last_fallback_heading_ = true;
    } else {
      const double alpha = clampd(fallback_heading_smooth_alpha_, 0.0, 0.99);
      const double err = wrap_pi(best_heading - last_fallback_heading_);
      last_fallback_heading_ = wrap_pi(last_fallback_heading_ + (1.0 - alpha) * err);
    }

    const double chosen_heading = last_fallback_heading_;

    const double usable_len = std::max(
      1.0,
      std::min(fallback_path_length_m_, best_clearance - fallback_clearance_margin_m_));

    nav_msgs::msg::Path out;
    out.header.stamp = this->now();
    out.header.frame_id = robot_frame_;

    const int n = std::max(2, static_cast<int>(std::floor(usable_len / fallback_point_spacing_m_)) + 1);
    out.poses.reserve(n);

    for (int i = 0; i < n; ++i) {
      const double s = std::min(usable_len, i * fallback_point_spacing_m_);

      geometry_msgs::msg::PoseStamped p;
      p.header = out.header;
      p.pose.position.x = s * std::cos(chosen_heading);
      p.pose.position.y = s * std::sin(chosen_heading);
      p.pose.position.z = 0.0;
      setYaw(p, chosen_heading);
      out.poses.push_back(p);
    }

    return out;
  }

  std::string robot_frame_;
  std::string guidance_topic_;
  std::string mode_topic_;

  std::string centerline_topic_;
  std::string left_boundary_topic_;
  std::string right_boundary_topic_;
  std::string lane_conf_topic_;
  std::string scan_topic_;

  double publish_hz_{10.0};
  double path_stale_sec_{0.40};
  int min_centerline_points_{5};
  int min_boundary_points_{5};

  double lane_conf_thresh_{0.45};
  double nominal_half_width_m_{2.20};
  bool prefer_centerline_when_available_{true};

  double fallback_path_length_m_{8.0};
  double fallback_point_spacing_m_{0.5};
  double fallback_max_heading_deg_{70.0};
  double fallback_heading_step_deg_{5.0};
  double fallback_clearance_margin_m_{0.9};
  double fallback_min_clear_range_m_{1.5};
  double fallback_scan_cap_m_{12.0};
  double fallback_heading_smooth_alpha_{0.85};

  bool smooth_output_path_{true};
  double path_smooth_alpha_{0.35};

  bool enable_turn_inside_bias_{true};
  double turn_detect_lookahead_idx_{6.0};
  double turn_detect_yaw_thresh_deg_{8.0};
  double inside_bias_fraction_{0.14};
  double inside_bias_max_m_{0.30};
  double inside_bias_fade_length_m_{4.5};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr centerline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr left_boundary_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr right_boundary_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lane_conf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr guidance_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;

  nav_msgs::msg::Path centerline_;
  nav_msgs::msg::Path left_boundary_;
  nav_msgs::msg::Path right_boundary_;
  sensor_msgs::msg::LaserScan last_scan_;

  double lane_confidence_{0.0};

  rclcpp::Time centerline_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time left_boundary_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time right_boundary_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time lane_confidence_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time scan_time_{0, 0, RCL_ROS_TIME};

  bool has_last_fallback_heading_{false};
  double last_fallback_heading_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuidancePathBuilder>());
  rclcpp::shutdown();
  return 0;
}