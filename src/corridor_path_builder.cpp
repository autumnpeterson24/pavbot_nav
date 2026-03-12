#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

struct Pt2
{
  double x{0.0};
  double y{0.0};
};

static inline double clampd(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

static inline double dist2d(const Pt2 & a, const Pt2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

static inline double norm2d(const Pt2 & p)
{
  return std::hypot(p.x, p.y);
}

static inline Pt2 add(const Pt2 & a, const Pt2 & b)
{
  return Pt2{a.x + b.x, a.y + b.y};
}

static inline Pt2 sub(const Pt2 & a, const Pt2 & b)
{
  return Pt2{a.x - b.x, a.y - b.y};
}

static inline Pt2 mul(const Pt2 & a, double s)
{
  return Pt2{a.x * s, a.y * s};
}

static inline geometry_msgs::msg::Quaternion quatFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

class CorridorPathBuilder : public rclcpp::Node
{
public:
  CorridorPathBuilder() : Node("corridor_path_builder")
  {
    declare_parameter<std::string>("left_boundary_topic", "/lanes/left_boundary");
    declare_parameter<std::string>("right_boundary_topic", "/lanes/right_boundary");
    declare_parameter<std::string>("lane_confidence_topic", "/lanes/confidence");
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("output_path_topic", "/corridor/path");
    declare_parameter<std::string>("output_mode_topic", "/corridor/mode");
    declare_parameter<std::string>("base_frame", "base_link");

    declare_parameter<double>("path_min_x_m", 1.0);
    declare_parameter<double>("path_max_x_m", 3.0);
    declare_parameter<double>("path_sample_dx_m", 0.4);

    declare_parameter<double>("nominal_half_width_m", 1.5);
    declare_parameter<bool>("auto_learn_half_width", false);
    declare_parameter<double>("half_width_learn_alpha", 0.95);
    declare_parameter<double>("half_width_min_m", 1.2);
    declare_parameter<double>("half_width_max_m", 1.8);

    declare_parameter<double>("path_smooth_alpha", 0.96);
    declare_parameter<double>("yaw_smooth_alpha", 0.92);
    declare_parameter<double>("path_hold_sec", 1.0);

    declare_parameter<double>("min_confidence_dual_wall", 0.50);
    declare_parameter<double>("min_confidence_single_wall", 0.20);

    declare_parameter<int>("min_boundary_points", 3);
    declare_parameter<double>("boundary_stale_sec", 0.8);
    declare_parameter<double>("scan_stale_sec", 0.5);
    declare_parameter<double>("max_center_abs_y_m", 0.8);
    declare_parameter<double>("max_interp_gap_m", 1.2);

    // LiDAR -> base_link planar offset (sim lidar sits forward of base origin)
    declare_parameter<double>("lidar_x_in_base_m", 0.60);
    declare_parameter<double>("lidar_y_in_base_m", 0.0);

    // Scan filtering
    declare_parameter<double>("scan_range_min_use_m", 0.20);
    declare_parameter<double>("scan_range_max_use_m", 8.0);
    declare_parameter<double>("scan_min_x_m", 0.40);
    declare_parameter<double>("scan_max_x_m", 4.0);
    declare_parameter<double>("scan_max_abs_y_m", 3.0);

    // Obstacle bias shaping
    declare_parameter<double>("obstacle_slice_half_width_m", 0.45);
    declare_parameter<double>("obstacle_influence_radius_m", 0.65);
    declare_parameter<double>("obstacle_repulsion_gain", 0.30);
    declare_parameter<double>("max_obstacle_bias_per_sample_m", 0.35);

    // Open-forward fallback
    declare_parameter<double>("open_forward_nominal_y_m", 0.0);
    declare_parameter<double>("open_forward_last_y_blend", 0.75);
    declare_parameter<double>("open_forward_smoothing_alpha", 0.85);

    declare_parameter<double>("publish_rate_hz", 10.0);
    declare_parameter<double>("debug_print_hz", 1.0);

    get_parameter("left_boundary_topic", left_boundary_topic_);
    get_parameter("right_boundary_topic", right_boundary_topic_);
    get_parameter("lane_confidence_topic", lane_conf_topic_);
    get_parameter("scan_topic", scan_topic_);
    get_parameter("output_path_topic", output_path_topic_);
    get_parameter("output_mode_topic", output_mode_topic_);
    get_parameter("base_frame", base_frame_);

    get_parameter("path_min_x_m", path_min_x_m_);
    get_parameter("path_max_x_m", path_max_x_m_);
    get_parameter("path_sample_dx_m", path_sample_dx_m_);

    get_parameter("nominal_half_width_m", nominal_half_width_m_);
    get_parameter("auto_learn_half_width", auto_learn_half_width_);
    get_parameter("half_width_learn_alpha", half_width_learn_alpha_);
    get_parameter("half_width_min_m", half_width_min_m_);
    get_parameter("half_width_max_m", half_width_max_m_);

    get_parameter("path_smooth_alpha", path_smooth_alpha_);
    get_parameter("yaw_smooth_alpha", yaw_smooth_alpha_);
    get_parameter("path_hold_sec", path_hold_sec_);

    get_parameter("min_confidence_dual_wall", min_conf_dual_);
    get_parameter("min_confidence_single_wall", min_conf_single_);

    get_parameter("min_boundary_points", min_boundary_points_);
    get_parameter("boundary_stale_sec", boundary_stale_sec_);
    get_parameter("scan_stale_sec", scan_stale_sec_);
    get_parameter("max_center_abs_y_m", max_center_abs_y_m_);
    get_parameter("max_interp_gap_m", max_interp_gap_m_);

    get_parameter("lidar_x_in_base_m", lidar_x_in_base_m_);
    get_parameter("lidar_y_in_base_m", lidar_y_in_base_m_);

    get_parameter("scan_range_min_use_m", scan_range_min_use_m_);
    get_parameter("scan_range_max_use_m", scan_range_max_use_m_);
    get_parameter("scan_min_x_m", scan_min_x_m_);
    get_parameter("scan_max_x_m", scan_max_x_m_);
    get_parameter("scan_max_abs_y_m", scan_max_abs_y_m_);

    get_parameter("obstacle_slice_half_width_m", obstacle_slice_half_width_m_);
    get_parameter("obstacle_influence_radius_m", obstacle_influence_radius_m_);
    get_parameter("obstacle_repulsion_gain", obstacle_repulsion_gain_);
    get_parameter("max_obstacle_bias_per_sample_m", max_obstacle_bias_per_sample_m_);

    get_parameter("open_forward_nominal_y_m", open_forward_nominal_y_m_);
    get_parameter("open_forward_last_y_blend", open_forward_last_y_blend_);
    get_parameter("open_forward_smoothing_alpha", open_forward_smoothing_alpha_);

    get_parameter("publish_rate_hz", publish_rate_hz_);
    get_parameter("debug_print_hz", debug_print_hz_);

    path_pub_ = create_publisher<nav_msgs::msg::Path>(output_path_topic_, 10);
    mode_pub_ = create_publisher<std_msgs::msg::String>(output_mode_topic_, 10);
    dbg_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/corridor/debug_points", 10);
    obs_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/corridor/obstacle_points", 10);

    left_sub_ = create_subscription<nav_msgs::msg::Path>(
      left_boundary_topic_, rclcpp::QoS(10),
      std::bind(&CorridorPathBuilder::onLeftBoundary, this, std::placeholders::_1));

    right_sub_ = create_subscription<nav_msgs::msg::Path>(
      right_boundary_topic_, rclcpp::QoS(10),
      std::bind(&CorridorPathBuilder::onRightBoundary, this, std::placeholders::_1));

    conf_sub_ = create_subscription<std_msgs::msg::Float32>(
      lane_conf_topic_, rclcpp::QoS(10),
      std::bind(&CorridorPathBuilder::onConfidence, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CorridorPathBuilder::onScan, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&CorridorPathBuilder::tick, this));

    debug_period_ms_ = (debug_print_hz_ > 0.0) ? static_cast<uint64_t>(1000.0 / debug_print_hz_) : 0;

    RCLCPP_INFO(get_logger(), "corridor_path_builder ready.");
  }

private:
  enum class Mode
  {
    EMPTY,
    HOLD_LAST,
    DUAL_WALL,
    LEFT_ONLY,
    RIGHT_ONLY,
    OPEN_FORWARD
  };

  struct BoundaryState
  {
    nav_msgs::msg::Path path;
    rclcpp::Time recv_time{0, 0, RCL_ROS_TIME};
    bool has{false};
  };

  struct ScanState
  {
    sensor_msgs::msg::LaserScan scan;
    rclcpp::Time recv_time{0, 0, RCL_ROS_TIME};
    bool has{false};
  };

  void onLeftBoundary(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    left_.path = *msg;
    left_.recv_time = now();
    left_.has = true;
  }

  void onRightBoundary(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    right_.path = *msg;
    right_.recv_time = now();
    right_.has = true;
  }

  void onConfidence(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    last_lane_conf_ = msg->data;
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    scan_.scan = *msg;
    scan_.recv_time = now();
    scan_.has = true;
  }

  static std::vector<Pt2> pathToPts(const nav_msgs::msg::Path & path)
  {
    std::vector<Pt2> pts;
    pts.reserve(path.poses.size());
    for (const auto & p : path.poses) {
      pts.push_back(Pt2{p.pose.position.x, p.pose.position.y});
    }
    return pts;
  }

  static std::vector<Pt2> sortByX(std::vector<Pt2> pts)
  {
    std::sort(pts.begin(), pts.end(), [](const Pt2 & a, const Pt2 & b) {
      return a.x < b.x;
    });

    std::vector<Pt2> out;
    out.reserve(pts.size());

    double last_x = -std::numeric_limits<double>::infinity();
    for (const auto & p : pts) {
      if (out.empty() || p.x > last_x + 1e-4) {
        out.push_back(p);
        last_x = p.x;
      } else if (!out.empty()) {
        out.back().y = 0.5 * (out.back().y + p.y);
      }
    }
    return out;
  }

  bool boundaryUsable(const BoundaryState & b) const
  {
    if (!b.has) return false;
    if ((now() - b.recv_time).seconds() > boundary_stale_sec_) return false;
    if ((int)b.path.poses.size() < min_boundary_points_) return false;
    return true;
  }

  bool scanUsable(const ScanState & s) const
  {
    if (!s.has) return false;
    if ((now() - s.recv_time).seconds() > scan_stale_sec_) return false;
    if (s.scan.ranges.empty()) return false;
    return true;
  }

  std::optional<double> interpYAtX(const std::vector<Pt2> & pts, double x_query) const
  {
    if (pts.size() < 2) return std::nullopt;
    if (x_query < pts.front().x || x_query > pts.back().x) return std::nullopt;

    for (size_t i = 1; i < pts.size(); ++i) {
      const Pt2 & p0 = pts[i - 1];
      const Pt2 & p1 = pts[i];
      if (x_query < p0.x || x_query > p1.x) continue;

      const double dx = p1.x - p0.x;
      if (dx < 1e-6) continue;
      if (dx > max_interp_gap_m_) return std::nullopt;

      const double t = (x_query - p0.x) / dx;
      return (1.0 - t) * p0.y + t * p1.y;
    }
    return std::nullopt;
  }

  double estimateHalfWidthFromDual(const std::vector<Pt2> & left_pts,
                                   const std::vector<Pt2> & right_pts) const
  {
    std::vector<double> samples;
    for (double x = path_min_x_m_; x <= path_max_x_m_ + 1e-6; x += path_sample_dx_m_) {
      auto yl = interpYAtX(left_pts, x);
      auto yr = interpYAtX(right_pts, x);
      if (!yl || !yr) continue;
      double hw = 0.5 * std::abs(*yl - *yr);
      if (std::isfinite(hw)) samples.push_back(hw);
    }

    if (samples.size() < 3) return nominal_half_width_m_;

    std::nth_element(samples.begin(), samples.begin() + samples.size() / 2, samples.end());
    double med = samples[samples.size() / 2];
    return clampd(med, half_width_min_m_, half_width_max_m_);
  }

  std::vector<Pt2> scanToObstaclePts(const sensor_msgs::msg::LaserScan & scan) const
  {
    std::vector<Pt2> obs;
    obs.reserve(scan.ranges.size());

    double ang = scan.angle_min;
    for (float rf : scan.ranges) {
      const double r = static_cast<double>(rf);
      if (!std::isfinite(r)) {
        ang += scan.angle_increment;
        continue;
      }

      if (r < std::max<double>(scan.range_min, scan_range_min_use_m_) ||
          r > std::min<double>(scan.range_max, scan_range_max_use_m_)) {
        ang += scan.angle_increment;
        continue;
      }

      // Assume lidar yaw aligned with base_link in sim; apply x/y offset only.
      const double x = lidar_x_in_base_m_ + r * std::cos(ang);
      const double y = lidar_y_in_base_m_ + r * std::sin(ang);

      if (x < scan_min_x_m_ || x > scan_max_x_m_) {
        ang += scan.angle_increment;
        continue;
      }
      if (std::abs(y) > scan_max_abs_y_m_) {
        ang += scan.angle_increment;
        continue;
      }

      obs.push_back(Pt2{x, y});
      ang += scan.angle_increment;
    }

    return obs;
  }

  double obstacleBiasAtX(double x_query, double y_seed, const std::vector<Pt2> & obs) const
  {
    double rep = 0.0;

    for (const auto & o : obs) {
      const double dx = std::abs(o.x - x_query);
      if (dx > obstacle_slice_half_width_m_) continue;

      const double dy = y_seed - o.y;
      const double ady = std::abs(dy);
      if (ady > obstacle_influence_radius_m_) continue;

      const double wx = 1.0 - clampd(dx / std::max(1e-6, obstacle_slice_half_width_m_), 0.0, 1.0);
      const double wy = 1.0 - clampd(ady / std::max(1e-6, obstacle_influence_radius_m_), 0.0, 1.0);
      const double w = wx * wy;

      double sign = 0.0;
      if (std::abs(dy) > 1e-4) {
        sign = (dy > 0.0) ? 1.0 : -1.0;
      } else {
        // If obstacle is almost exactly at seed y, push away from obstacle side relative to center.
        sign = (o.y >= 0.0) ? -1.0 : 1.0;
      }

      rep += sign * w;
    }

    return clampd(obstacle_repulsion_gain_ * rep,
                  -max_obstacle_bias_per_sample_m_,
                  +max_obstacle_bias_per_sample_m_);
  }

  std::vector<Pt2> applyObstacleBias(const std::vector<Pt2> & in,
                                     const std::vector<Pt2> & obs) const
  {
    if (in.empty() || obs.empty()) return in;

    std::vector<Pt2> out = in;
    for (auto & p : out) {
      const double bias = obstacleBiasAtX(p.x, p.y, obs);
      p.y = clampd(p.y + bias, -max_center_abs_y_m_, +max_center_abs_y_m_);
    }
    return out;
  }

  std::vector<Pt2> buildDualWallPath(const std::vector<Pt2> & left_pts,
                                     const std::vector<Pt2> & right_pts) const
  {
    std::vector<Pt2> out;
    for (double x = path_min_x_m_; x <= path_max_x_m_ + 1e-6; x += path_sample_dx_m_) {
      auto yl = interpYAtX(left_pts, x);
      auto yr = interpYAtX(right_pts, x);
      if (!yl || !yr) continue;

      Pt2 c{x, 0.5 * (*yl + *yr)};
      c.y = clampd(c.y, -max_center_abs_y_m_, +max_center_abs_y_m_);
      out.push_back(c);
    }
    return out;
  }

  std::vector<Pt2> buildSingleWallPathLeft(const std::vector<Pt2> & left_pts, double half_width) const
  {
    std::vector<Pt2> out;
    for (double x = path_min_x_m_; x <= path_max_x_m_ + 1e-6; x += path_sample_dx_m_) {
      auto yl = interpYAtX(left_pts, x);
      if (!yl) continue;

      Pt2 c{x, *yl - half_width};
      c.y = clampd(c.y, -max_center_abs_y_m_, +max_center_abs_y_m_);
      out.push_back(c);
    }
    return out;
  }

  std::vector<Pt2> buildSingleWallPathRight(const std::vector<Pt2> & right_pts, double half_width) const
  {
    std::vector<Pt2> out;
    for (double x = path_min_x_m_; x <= path_max_x_m_ + 1e-6; x += path_sample_dx_m_) {
      auto yr = interpYAtX(right_pts, x);
      if (!yr) continue;

      Pt2 c{x, *yr + half_width};
      c.y = clampd(c.y, -max_center_abs_y_m_, +max_center_abs_y_m_);
      out.push_back(c);
    }
    return out;
  }

  std::vector<Pt2> buildOpenForwardPath(const std::vector<Pt2> & obs) const
  {
    std::vector<Pt2> out;

    double last_y = open_forward_nominal_y_m_;
    if (has_last_published_path_ && !last_published_path_.poses.empty()) {
      last_y = last_published_path_.poses.front().pose.position.y;
    }

    double y_curr =
      open_forward_last_y_blend_ * last_y +
      (1.0 - open_forward_last_y_blend_) * open_forward_nominal_y_m_;
    y_curr = clampd(y_curr, -max_center_abs_y_m_, +max_center_abs_y_m_);

    for (double x = path_min_x_m_; x <= path_max_x_m_ + 1e-6; x += path_sample_dx_m_) {
      const double bias = obstacleBiasAtX(x, y_curr, obs);
      const double y_des = clampd(y_curr + bias, -max_center_abs_y_m_, +max_center_abs_y_m_);

      y_curr =
        open_forward_smoothing_alpha_ * y_curr +
        (1.0 - open_forward_smoothing_alpha_) * y_des;
      y_curr = clampd(y_curr, -max_center_abs_y_m_, +max_center_abs_y_m_);

      out.push_back(Pt2{x, y_curr});
    }

    return out;
  }

  std::vector<Pt2> smoothPathPts(const std::vector<Pt2> & pts)
  {
    if (pts.empty()) return pts;
    if (!has_last_path_pts_) {
      last_path_pts_ = pts;
      has_last_path_pts_ = true;
      return pts;
    }

    std::vector<Pt2> out = pts;
    const double a = clampd(path_smooth_alpha_, 0.0, 0.99);
    const size_t N = std::min(out.size(), last_path_pts_.size());

    for (size_t i = 0; i < N; ++i) {
      out[i].x = pts[i].x;
      out[i].y = a * last_path_pts_[i].y + (1.0 - a) * pts[i].y;
    }

    last_path_pts_ = out;
    return out;
  }

  nav_msgs::msg::Path ptsToPath(const std::vector<Pt2> & pts, const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = base_frame_;
    path.header.stamp = stamp;

    if (pts.empty()) return path;

    std::vector<double> yaws(pts.size(), 0.0);
    for (size_t i = 0; i < pts.size(); ++i) {
      Pt2 t;
      if (i == 0 && pts.size() > 1) {
        t = sub(pts[1], pts[0]);
      } else if (i == pts.size() - 1 && pts.size() > 1) {
        t = sub(pts[i], pts[i - 1]);
      } else if (pts.size() > 2) {
        t = sub(pts[i + 1], pts[i - 1]);
      } else {
        t = Pt2{1.0, 0.0};
      }

      double yaw = std::atan2(t.y, t.x);
      if (!has_last_yaw_) {
        last_yaw_ = yaw;
        has_last_yaw_ = true;
      } else if (i == 0) {
        const double a = clampd(yaw_smooth_alpha_, 0.0, 0.99);
        last_yaw_ = a * last_yaw_ + (1.0 - a) * yaw;
        yaw = last_yaw_;
      }
      yaws[i] = yaw;
    }

    for (size_t i = 0; i < pts.size(); ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = pts[i].x;
      p.pose.position.y = pts[i].y;
      p.pose.position.z = 0.0;
      p.pose.orientation = quatFromYaw(yaws[i]);
      path.poses.push_back(p);
    }
    return path;
  }

  void publishMode(Mode m)
  {
    std_msgs::msg::String s;
    switch (m) {
      case Mode::EMPTY: s.data = "EMPTY"; break;
      case Mode::HOLD_LAST: s.data = "HOLD_LAST"; break;
      case Mode::DUAL_WALL: s.data = "DUAL_WALL"; break;
      case Mode::LEFT_ONLY: s.data = "LEFT_ONLY"; break;
      case Mode::RIGHT_ONLY: s.data = "RIGHT_ONLY"; break;
      case Mode::OPEN_FORWARD: s.data = "OPEN_FORWARD"; break;
    }
    mode_pub_->publish(s);
  }

  void publishDebugMarker(const std::vector<Pt2> & pts, const rclcpp::Time & stamp, Mode mode)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.header.stamp = stamp;
    m.ns = "corridor_path_builder";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.12;
    m.scale.y = 0.12;
    m.scale.z = 0.12;

    switch (mode) {
      case Mode::DUAL_WALL:
        m.color.g = 1.0; m.color.a = 1.0;
        break;
      case Mode::LEFT_ONLY:
      case Mode::RIGHT_ONLY:
        m.color.r = 1.0; m.color.g = 0.75; m.color.a = 1.0;
        break;
      case Mode::OPEN_FORWARD:
        m.color.g = 1.0; m.color.b = 1.0; m.color.a = 1.0;
        break;
      case Mode::HOLD_LAST:
        m.color.b = 1.0; m.color.a = 1.0;
        break;
      case Mode::EMPTY:
      default:
        m.color.r = 1.0; m.color.a = 0.6;
        break;
    }

    for (const auto & p : pts) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x;
      gp.y = p.y;
      gp.z = 0.05;
      m.points.push_back(gp);
    }

    dbg_marker_pub_->publish(m);
  }

  void publishObstacleMarker(const std::vector<Pt2> & obs, const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.header.stamp = stamp;
    m.ns = "corridor_obstacles";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.10;
    m.scale.y = 0.10;
    m.scale.z = 0.10;
    m.color.r = 1.0;
    m.color.g = 0.2;
    m.color.b = 0.2;
    m.color.a = 0.9;

    for (const auto & o : obs) {
      geometry_msgs::msg::Point gp;
      gp.x = o.x;
      gp.y = o.y;
      gp.z = 0.05;
      m.points.push_back(gp);
    }

    obs_marker_pub_->publish(m);
  }

  bool forwardMonotonicEnough(const std::vector<Pt2> & pts) const
  {
    if (pts.size() < 2) return false;
    int good = 0;
    for (size_t i = 1; i < pts.size(); ++i) {
      if (pts[i].x > pts[i - 1].x + 1e-3) good++;
    }
    return good >= static_cast<int>(pts.size()) - 1;
  }

  void publishEmpty(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Path empty;
    empty.header.frame_id = base_frame_;
    empty.header.stamp = stamp;
    path_pub_->publish(empty);
    publishMode(Mode::EMPTY);
    publishDebugMarker({}, stamp, Mode::EMPTY);
  }

  void tick()
  {
    BoundaryState left, right;
    ScanState scan;
    float lane_conf = 0.0f;

    {
      std::lock_guard<std::mutex> lk(mutex_);
      left = left_;
      right = right_;
      scan = scan_;
      lane_conf = last_lane_conf_;
    }

    const bool left_ok = boundaryUsable(left);
    const bool right_ok = boundaryUsable(right);
    const bool scan_ok = scanUsable(scan);

    std::vector<Pt2> left_pts = left_ok ? sortByX(pathToPts(left.path)) : std::vector<Pt2>{};
    std::vector<Pt2> right_pts = right_ok ? sortByX(pathToPts(right.path)) : std::vector<Pt2>{};
    std::vector<Pt2> obs_pts = scan_ok ? scanToObstaclePts(scan.scan) : std::vector<Pt2>{};

    if (auto_learn_half_width_ && left_ok && right_ok) {
      double hw_est = estimateHalfWidthFromDual(left_pts, right_pts);
      const double a = clampd(half_width_learn_alpha_, 0.0, 0.99);
      nominal_half_width_m_ = a * nominal_half_width_m_ + (1.0 - a) * hw_est;
      nominal_half_width_m_ = clampd(nominal_half_width_m_, half_width_min_m_, half_width_max_m_);
    }

    Mode mode = Mode::EMPTY;
    std::vector<Pt2> corridor_pts;

    const bool dual_allowed = left_ok && right_ok && lane_conf >= min_conf_dual_;
    const bool left_single_allowed = left_ok && (!right_ok || lane_conf >= min_conf_single_);
    const bool right_single_allowed = right_ok && (!left_ok || lane_conf >= min_conf_single_);

    if (dual_allowed) {
      corridor_pts = buildDualWallPath(left_pts, right_pts);
      mode = Mode::DUAL_WALL;
    } else if (left_single_allowed && !right_ok) {
      corridor_pts = buildSingleWallPathLeft(left_pts, nominal_half_width_m_);
      mode = Mode::LEFT_ONLY;
    } else if (right_single_allowed && !left_ok) {
      corridor_pts = buildSingleWallPathRight(right_pts, nominal_half_width_m_);
      mode = Mode::RIGHT_ONLY;
    } else if (scan_ok) {
      corridor_pts = buildOpenForwardPath(obs_pts);
      mode = Mode::OPEN_FORWARD;
    }

    if (!corridor_pts.empty()) {
      corridor_pts = sortByX(corridor_pts);
      corridor_pts = applyObstacleBias(corridor_pts, obs_pts);
    }

    if (!corridor_pts.empty() && forwardMonotonicEnough(corridor_pts)) {
      corridor_pts = smoothPathPts(corridor_pts);
      auto path_msg = ptsToPath(corridor_pts, now());
      path_pub_->publish(path_msg);
      publishMode(mode);
      publishDebugMarker(corridor_pts, path_msg.header.stamp, mode);
      publishObstacleMarker(obs_pts, path_msg.header.stamp);

      last_published_path_ = path_msg;
      last_publish_time_ = now();
      has_last_published_path_ = true;

      if (debug_period_ms_ > 0) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), debug_period_ms_,
          "mode=%s lane_conf=%.2f left_ok=%d right_ok=%d scan_ok=%d obs=%zu half_width=%.2f points=%zu",
          mode == Mode::DUAL_WALL ? "DUAL_WALL" :
          mode == Mode::LEFT_ONLY ? "LEFT_ONLY" :
          mode == Mode::RIGHT_ONLY ? "RIGHT_ONLY" :
          mode == Mode::OPEN_FORWARD ? "OPEN_FORWARD" : "OTHER",
          lane_conf,
          left_ok ? 1 : 0,
          right_ok ? 1 : 0,
          scan_ok ? 1 : 0,
          obs_pts.size(),
          nominal_half_width_m_,
          corridor_pts.size());
      }
      return;
    }

    const double hold_age = has_last_published_path_ ? (now() - last_publish_time_).seconds() : 1e9;
    if (has_last_published_path_ && hold_age <= path_hold_sec_) {
      auto held = last_published_path_;
      held.header.stamp = now();
      path_pub_->publish(held);
      publishMode(Mode::HOLD_LAST);

      auto held_pts = pathToPts(held);
      publishDebugMarker(held_pts, held.header.stamp, Mode::HOLD_LAST);
      publishObstacleMarker(obs_pts, held.header.stamp);

      if (debug_period_ms_ > 0) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), debug_period_ms_,
          "mode=HOLD_LAST hold_age=%.2f lane_conf=%.2f left_ok=%d right_ok=%d scan_ok=%d",
          hold_age, lane_conf, left_ok ? 1 : 0, right_ok ? 1 : 0, scan_ok ? 1 : 0);
      }
      return;
    }

    publishEmpty(now());

    if (debug_period_ms_ > 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), debug_period_ms_,
        "mode=EMPTY lane_conf=%.2f left_ok=%d right_ok=%d scan_ok=%d",
        lane_conf, left_ok ? 1 : 0, right_ok ? 1 : 0, scan_ok ? 1 : 0);
    }
  }

  std::string left_boundary_topic_;
  std::string right_boundary_topic_;
  std::string lane_conf_topic_;
  std::string scan_topic_;
  std::string output_path_topic_;
  std::string output_mode_topic_;
  std::string base_frame_;

  double path_min_x_m_{1.0};
  double path_max_x_m_{3.0};
  double path_sample_dx_m_{0.4};

  double nominal_half_width_m_{1.5};
  bool auto_learn_half_width_{false};
  double half_width_learn_alpha_{0.95};
  double half_width_min_m_{1.2};
  double half_width_max_m_{1.8};

  double path_smooth_alpha_{0.96};
  double yaw_smooth_alpha_{0.92};
  double path_hold_sec_{1.0};

  double min_conf_dual_{0.50};
  double min_conf_single_{0.20};

  int min_boundary_points_{3};
  double boundary_stale_sec_{0.8};
  double scan_stale_sec_{0.5};
  double max_center_abs_y_m_{0.8};
  double max_interp_gap_m_{1.2};

  double lidar_x_in_base_m_{0.60};
  double lidar_y_in_base_m_{0.0};

  double scan_range_min_use_m_{0.20};
  double scan_range_max_use_m_{8.0};
  double scan_min_x_m_{0.40};
  double scan_max_x_m_{4.0};
  double scan_max_abs_y_m_{3.0};

  double obstacle_slice_half_width_m_{0.45};
  double obstacle_influence_radius_m_{0.65};
  double obstacle_repulsion_gain_{0.30};
  double max_obstacle_bias_per_sample_m_{0.35};

  double open_forward_nominal_y_m_{0.0};
  double open_forward_last_y_blend_{0.75};
  double open_forward_smoothing_alpha_{0.85};

  double publish_rate_hz_{10.0};
  double debug_print_hz_{1.0};
  uint64_t debug_period_ms_{1000};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr left_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr right_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr conf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr dbg_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obs_marker_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  BoundaryState left_;
  BoundaryState right_;
  ScanState scan_;
  float last_lane_conf_{0.0f};

  std::vector<Pt2> last_path_pts_;
  bool has_last_path_pts_{false};

  nav_msgs::msg::Path last_published_path_;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  bool has_last_published_path_{false};

  double last_yaw_{0.0};
  bool has_last_yaw_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CorridorPathBuilder>());
  rclcpp::shutdown();
  return 0;
}