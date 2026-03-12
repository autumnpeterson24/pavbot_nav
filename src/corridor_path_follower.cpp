#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <cmath>
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

static inline double wrapPi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static inline double norm2d(const Pt2 & p)
{
  return std::hypot(p.x, p.y);
}

static inline double dist2d(const Pt2 & a, const Pt2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

class CorridorPathFollower : public rclcpp::Node
{
public:
  CorridorPathFollower() : Node("corridor_path_follower")
  {
    declare_parameter<std::string>("path_topic", "/corridor/path");
    declare_parameter<std::string>("mode_topic", "/corridor/mode");
    declare_parameter<std::string>("cmd_vel_topic", "cmd_vel_nav");
    declare_parameter<std::string>("base_frame", "base_link");

    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("path_stale_sec", 0.40);
    declare_parameter<int>("min_path_points", 3);

    declare_parameter<double>("lookahead_dist_m", 1.5);
    declare_parameter<double>("min_lookahead_dist_m", 0.8);
    declare_parameter<double>("max_lookahead_dist_m", 2.5);

    declare_parameter<double>("max_linear_speed_mps", 1.2);
    declare_parameter<double>("min_linear_speed_mps", 0.15);
    declare_parameter<double>("max_angular_speed_radps", 1.2);

    declare_parameter<double>("k_ang", 1.8);
    declare_parameter<double>("k_lat", 1.2);

    declare_parameter<double>("slowdown_yaw_error_rad", 0.35);
    declare_parameter<double>("stop_yaw_error_rad", 1.10);
    declare_parameter<double>("slowdown_lateral_error_m", 0.35);
    declare_parameter<double>("stop_lateral_error_m", 1.0);

    declare_parameter<double>("path_hold_stop_sec", 0.60);
    declare_parameter<double>("goal_taper_dist_m", 1.5);
    declare_parameter<double>("stop_at_path_end_dist_m", 0.40);

    declare_parameter<double>("cmd_smooth_alpha", 0.75);
    declare_parameter<bool>("publish_debug_marker", true);
    declare_parameter<double>("debug_print_hz", 2.0);

    get_parameter("path_topic", path_topic_);
    get_parameter("mode_topic", mode_topic_);
    get_parameter("cmd_vel_topic", cmd_vel_topic_);
    get_parameter("base_frame", base_frame_);

    get_parameter("control_rate_hz", control_rate_hz_);
    get_parameter("path_stale_sec", path_stale_sec_);
    get_parameter("min_path_points", min_path_points_);

    get_parameter("lookahead_dist_m", lookahead_dist_m_);
    get_parameter("min_lookahead_dist_m", min_lookahead_dist_m_);
    get_parameter("max_lookahead_dist_m", max_lookahead_dist_m_);

    get_parameter("max_linear_speed_mps", max_linear_speed_mps_);
    get_parameter("min_linear_speed_mps", min_linear_speed_mps_);
    get_parameter("max_angular_speed_radps", max_angular_speed_radps_);

    get_parameter("k_ang", k_ang_);
    get_parameter("k_lat", k_lat_);

    get_parameter("slowdown_yaw_error_rad", slowdown_yaw_error_rad_);
    get_parameter("stop_yaw_error_rad", stop_yaw_error_rad_);
    get_parameter("slowdown_lateral_error_m", slowdown_lateral_error_m_);
    get_parameter("stop_lateral_error_m", stop_lateral_error_m_);

    get_parameter("path_hold_stop_sec", path_hold_stop_sec_);
    get_parameter("goal_taper_dist_m", goal_taper_dist_m_);
    get_parameter("stop_at_path_end_dist_m", stop_at_path_end_dist_m_);

    get_parameter("cmd_smooth_alpha", cmd_smooth_alpha_);
    get_parameter("publish_debug_marker", publish_debug_marker_);
    get_parameter("debug_print_hz", debug_print_hz_);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10),
      std::bind(&CorridorPathFollower::onPath, this, std::placeholders::_1));

    mode_sub_ = create_subscription<std_msgs::msg::String>(
      mode_topic_, rclcpp::QoS(10),
      std::bind(&CorridorPathFollower::onMode, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    debug_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/corridor/follower_lookahead", 10);

    srv_ = create_service<std_srvs::srv::SetBool>(
      "/autonomy/set_enabled",
      std::bind(&CorridorPathFollower::onSetEnabled, this, std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&CorridorPathFollower::tick, this));

    debug_period_ms_ = (debug_print_hz_ > 0.0) ? static_cast<uint64_t>(1000.0 / debug_print_hz_) : 0;

    RCLCPP_INFO(get_logger(), "corridor_path_follower ready.");
    RCLCPP_INFO(get_logger(), "Enable with:");
    RCLCPP_INFO(get_logger(), "ros2 service call /autonomy/set_enabled std_srvs/srv/SetBool \"{data: true}\"");
  }

private:
  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    last_path_ = *msg;
    last_path_recv_time_ = now();
    has_path_ = true;
  }

  void onMode(const std_msgs::msg::String::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    last_mode_ = msg->data;
  }

  void onSetEnabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    enabled_ = req->data;
    if (!enabled_) {
      publishStop();
      res->success = true;
      res->message = "Corridor follower disabled.";
      RCLCPP_INFO(get_logger(), "Corridor follower DISABLED");
    } else {
      res->success = true;
      res->message = "Corridor follower enabled.";
      RCLCPP_INFO(get_logger(), "Corridor follower ENABLED");
    }
  }

  std::vector<Pt2> pathToPts(const nav_msgs::msg::Path & path) const
  {
    std::vector<Pt2> pts;
    pts.reserve(path.poses.size());
    for (const auto & p : path.poses) {
      pts.push_back(Pt2{p.pose.position.x, p.pose.position.y});
    }
    return pts;
  }

  double pathLength(const std::vector<Pt2> & pts) const
  {
    if (pts.size() < 2) return 0.0;
    double L = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
      L += dist2d(pts[i - 1], pts[i]);
    }
    return L;
  }

  std::optional<Pt2> pickLookaheadPoint(const std::vector<Pt2> & pts, double lookahead) const
  {
    if (pts.empty()) return std::nullopt;
    if (pts.size() == 1) return pts.front();

    double acc = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
      const double ds = dist2d(pts[i - 1], pts[i]);
      acc += ds;
      if (acc >= lookahead) {
        return pts[i];
      }
    }
    return pts.back();
  }

  double estimatePathHeading(const std::vector<Pt2> & pts) const
  {
    if (pts.size() < 2) return 0.0;

    size_t i0 = 0;
    size_t i1 = std::min<size_t>(2, pts.size() - 1);

    Pt2 d{pts[i1].x - pts[i0].x, pts[i1].y - pts[i0].y};
    if (norm2d(d) < 1e-6) return 0.0;
    return std::atan2(d.y, d.x);
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
    last_cmd_lin_ = 0.0;
    last_cmd_ang_ = 0.0;
  }

  void publishLookaheadMarker(const Pt2 & p)
  {
    if (!publish_debug_marker_) return;

    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.header.stamp = now();
    m.ns = "corridor_path_follower";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = p.x;
    m.pose.position.y = p.y;
    m.pose.position.z = 0.08;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.20;
    m.scale.y = 0.20;
    m.scale.z = 0.20;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.a = 1.0;
    debug_marker_pub_->publish(m);
  }

  void tick()
  {
    if (!enabled_) {
      publishStop();
      return;
    }

    nav_msgs::msg::Path path_msg;
    std::string mode;
    rclcpp::Time recv_time{0, 0, RCL_ROS_TIME};
    bool has_path = false;

    {
      std::lock_guard<std::mutex> lk(mutex_);
      path_msg = last_path_;
      mode = last_mode_;
      recv_time = last_path_recv_time_;
      has_path = has_path_;
    }

    if (!has_path) {
      publishStop();
      return;
    }

    const double age = (now() - recv_time).seconds();
    if (age > path_hold_stop_sec_) {
      publishStop();
      return;
    }

    auto pts = pathToPts(path_msg);
    if ((int)pts.size() < min_path_points_) {
      if (age > path_stale_sec_) {
        publishStop();
      } else {
        publishStop();
      }
      return;
    }

    const double total_path_len = pathLength(pts);
    if (total_path_len < stop_at_path_end_dist_m_) {
      publishStop();
      return;
    }

    double lookahead = clampd(lookahead_dist_m_, min_lookahead_dist_m_, max_lookahead_dist_m_);
    auto lookahead_pt_opt = pickLookaheadPoint(pts, lookahead);
    if (!lookahead_pt_opt) {
      publishStop();
      return;
    }

    const Pt2 lookahead_pt = *lookahead_pt_opt;
    publishLookaheadMarker(lookahead_pt);

    const double heading_to_lookahead = std::atan2(lookahead_pt.y, lookahead_pt.x);
    const double path_heading = estimatePathHeading(pts);
    const double yaw_error = wrapPi(heading_to_lookahead);
    const double lat_error = lookahead_pt.y;

    double ang_cmd = k_ang_ * yaw_error + k_lat_ * lat_error;
    ang_cmd = clampd(ang_cmd, -max_angular_speed_radps_, max_angular_speed_radps_);

    double lin_cmd = max_linear_speed_mps_;

    // Slow down for heading error
    {
      const double ay = std::abs(yaw_error);
      if (ay >= stop_yaw_error_rad_) {
        lin_cmd = 0.0;
      } else if (ay > slowdown_yaw_error_rad_) {
        const double t = (ay - slowdown_yaw_error_rad_) /
                         std::max(1e-6, stop_yaw_error_rad_ - slowdown_yaw_error_rad_);
        lin_cmd *= (1.0 - 0.85 * clampd(t, 0.0, 1.0));
      }
    }

    // Slow down for lateral error
    {
      const double al = std::abs(lat_error);
      if (al >= stop_lateral_error_m_) {
        lin_cmd = 0.0;
      } else if (al > slowdown_lateral_error_m_) {
        const double t = (al - slowdown_lateral_error_m_) /
                         std::max(1e-6, stop_lateral_error_m_ - slowdown_lateral_error_m_);
        lin_cmd *= (1.0 - 0.75 * clampd(t, 0.0, 1.0));
      }
    }

    // Slow down near short path end
    if (total_path_len < goal_taper_dist_m_) {
      const double t = clampd(total_path_len / std::max(1e-6, goal_taper_dist_m_), 0.0, 1.0);
      lin_cmd *= t;
    }

    // If path mode is weaker than dual wall, be a little more conservative
    if (mode == "LEFT_ONLY" || mode == "RIGHT_ONLY") {
      lin_cmd *= 0.75;
    } else if (mode == "HOLD_LAST") {
      lin_cmd *= 0.55;
    }

    if (lin_cmd > 0.0) {
      lin_cmd = std::max(lin_cmd, min_linear_speed_mps_);
    }

    // Smooth output commands
    const double a = clampd(cmd_smooth_alpha_, 0.0, 0.99);
    const double lin_out = a * last_cmd_lin_ + (1.0 - a) * lin_cmd;
    const double ang_out = a * last_cmd_ang_ + (1.0 - a) * ang_cmd;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = lin_out;
    cmd.angular.z = ang_out;
    cmd_pub_->publish(cmd);

    last_cmd_lin_ = lin_out;
    last_cmd_ang_ = ang_out;

    if (debug_period_ms_ > 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), debug_period_ms_,
        "mode=%s age=%.2f pts=%zu lookahead=(%.2f, %.2f) path_len=%.2f yaw_err=%.2f path_head=%.2f lin=%.2f ang=%.2f",
        mode.c_str(),
        age,
        pts.size(),
        lookahead_pt.x, lookahead_pt.y,
        total_path_len,
        yaw_error,
        path_heading,
        lin_out,
        ang_out);
    }
  }

  std::string path_topic_;
  std::string mode_topic_;
  std::string cmd_vel_topic_;
  std::string base_frame_;

  double control_rate_hz_{20.0};
  double path_stale_sec_{0.40};
  int min_path_points_{3};

  double lookahead_dist_m_{1.5};
  double min_lookahead_dist_m_{0.8};
  double max_lookahead_dist_m_{2.5};

  double max_linear_speed_mps_{1.2};
  double min_linear_speed_mps_{0.15};
  double max_angular_speed_radps_{1.2};

  double k_ang_{1.8};
  double k_lat_{1.2};

  double slowdown_yaw_error_rad_{0.35};
  double stop_yaw_error_rad_{1.10};
  double slowdown_lateral_error_m_{0.35};
  double stop_lateral_error_m_{1.0};

  double path_hold_stop_sec_{0.60};
  double goal_taper_dist_m_{1.5};
  double stop_at_path_end_dist_m_{0.40};

  double cmd_smooth_alpha_{0.75};
  bool publish_debug_marker_{true};
  double debug_print_hz_{2.0};
  uint64_t debug_period_ms_{500};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_marker_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  nav_msgs::msg::Path last_path_;
  rclcpp::Time last_path_recv_time_{0, 0, RCL_ROS_TIME};
  std::string last_mode_{"EMPTY"};
  bool has_path_{false};

  bool enabled_{false};

  double last_cmd_lin_{0.0};
  double last_cmd_ang_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CorridorPathFollower>());
  rclcpp::shutdown();
  return 0;
}
