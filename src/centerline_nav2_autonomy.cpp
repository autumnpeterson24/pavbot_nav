#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class CenterlineNav2Autonomy : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  CenterlineNav2Autonomy()
  : Node("centerline_nav2_autonomy"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Parameters
    this->declare_parameter<std::string>("centerline_topic", "/lanes/centerline");
    this->declare_parameter<std::string>("global_frame", "odom");
    this->declare_parameter<std::string>("robot_frame", "base_link");
    this->declare_parameter<double>("lookahead_m", 4.0);
    this->declare_parameter<double>("goal_update_hz", 10.0); //change to make higher
    this->declare_parameter<double>("min_goal_separation_m", 0.2);
    this->declare_parameter<double>("path_stale_sec", 0.25);

    centerline_topic_ = this->get_parameter("centerline_topic").as_string();
    global_frame_ = this->get_parameter("global_frame").as_string();
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    lookahead_m_ = this->get_parameter("lookahead_m").as_double();
    goal_update_hz_ = this->get_parameter("goal_update_hz").as_double();
    min_goal_sep_m_ = this->get_parameter("min_goal_separation_m").as_double();
    path_stale_sec_ = this->get_parameter("path_stale_sec").as_double();

    // Sub to centerline path
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      centerline_topic_, rclcpp::QoS(10),
      std::bind(&CenterlineNav2Autonomy::onPath, this, std::placeholders::_1));

    // Nav2 action client
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Enable/disable service
    srv_ = this->create_service<std_srvs::srv::SetBool>(
      "/autonomy/set_enabled",
      std::bind(&CenterlineNav2Autonomy::onSetEnabled, this, std::placeholders::_1, std::placeholders::_2));

    // Timer (does nothing unless enabled)
    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, goal_update_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&CenterlineNav2Autonomy::tick, this));

    RCLCPP_INFO(this->get_logger(), "Centerline autonomy node ready (idle). Use /autonomy/set_enabled.");
  }

private:
  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    last_path_ = *msg;
    last_path_time_ = this->now();
  }

  void onSetEnabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    if (req->data) {
      enabled_ = true;
      res->success = true;
      res->message = "Autonomy enabled.";
      RCLCPP_INFO(this->get_logger(), "Autonomy ENABLED");
    } else {
      enabled_ = false;
      cancelCurrentGoal();
      res->success = true;
      res->message = "Autonomy disabled and goal cancelled.";
      RCLCPP_INFO(this->get_logger(), "Autonomy DISABLED (goal cancelled)");
    }
  }

  void tick()
  {
    if (!enabled_) {
      return;
    }

    // Wait for Nav2 server
    if (!nav_client_->wait_for_action_server(0s)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for Nav2 action server 'navigate_to_pose'...");
      return;
    }

    nav_msgs::msg::Path path_copy;
    rclcpp::Time path_time;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      path_copy = last_path_;
      path_time = last_path_time_;
    }

    if (path_copy.poses.size() < 5) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Centerline path too short / not received yet.");
      return;
    }

    // Ensure path is fresh
    const double age = (this->now() - path_time).seconds();
    if (age > path_stale_sec_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Centerline path stale (age=%.2fs).", age);
      return;
    }

    // Compute a goal pose in robot_frame by walking along the path arc length
    auto goal_in_path_frame = pickLookaheadPose(path_copy, lookahead_m_);
    if (!goal_in_path_frame) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Failed to pick a lookahead pose.");
      return;
    }

    // Transform goal into global_frame for Nav2
    geometry_msgs::msg::PoseStamped goal_global;
    try {
      goal_global = tf_buffer_.transform(*goal_in_path_frame, global_frame_, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF transform failed: %s", ex.what());
      return;
    }

    // Don’t spam goals if the new goal is basically the same
    if (has_last_goal_) {
      const double dx = goal_global.pose.position.x - last_goal_global_.pose.position.x;
      const double dy = goal_global.pose.position.y - last_goal_global_.pose.position.y;
      if (std::hypot(dx, dy) < min_goal_sep_m_) {
        return;
      }
    }

    sendGoal(goal_global);
    last_goal_global_ = goal_global;
    has_last_goal_ = true;
  }

  std::optional<geometry_msgs::msg::PoseStamped> pickLookaheadPose(
    const nav_msgs::msg::Path & path, double lookahead_m)
  {
    if (path.poses.size() < 2) return std::nullopt;

    // We assume the path frame is robot-relative (base_link). If it's already base_link, great.
    // Walk forward along the sequence and accumulate distance.
    double acc = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & p0 = path.poses[i - 1].pose.position;
      const auto & p1 = path.poses[i].pose.position;
      const double ds = std::hypot(p1.x - p0.x, p1.y - p0.y);
      acc += ds;
      if (acc >= lookahead_m) {
        auto goal = path.poses[i];
        // Ensure stamp is "now" to avoid TF-time complaints
        goal.header.stamp = this->now();
        return goal;
      }
    }

    // If path is shorter than lookahead, use the last pose
    auto goal = path.poses.back();
    goal.header.stamp = this->now();
    return goal;
  }

  void sendGoal(const geometry_msgs::msg::PoseStamped & goal)
  {
    NavigateToPose::Goal nav_goal;
    nav_goal.pose = goal;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](const GoalHandleNav::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_WARN(this->get_logger(), "Nav2 rejected goal.");
        } else {
          RCLCPP_INFO(this->get_logger(), "Nav2 accepted goal.");
          current_goal_handle_ = handle;
        }
      };

    send_goal_options.result_callback =
      [this](const GoalHandleNav::WrappedResult & result) {
        (void)result;
        // We keep streaming goals, so result is informational.
      };

    nav_client_->async_send_goal(nav_goal, send_goal_options);
  }

  void cancelCurrentGoal()
  {
    if (current_goal_handle_) {
      nav_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_.reset();
    }
    has_last_goal_ = false;
  }

  // Params
  std::string centerline_topic_;
  std::string global_frame_;
  std::string robot_frame_;
  double lookahead_m_{6.0};
  double goal_update_hz_{2.0};
  double min_goal_sep_m_{1.0};
  double path_stale_sec_{0.5};

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNav::SharedPtr current_goal_handle_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // State
  std::mutex mutex_;
  nav_msgs::msg::Path last_path_;
  rclcpp::Time last_path_time_{0, 0, RCL_ROS_TIME};

  bool enabled_{false};
  bool has_last_goal_{false};
  geometry_msgs::msg::PoseStamped last_goal_global_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CenterlineNav2Autonomy>());
  rclcpp::shutdown();
  return 0;
}
