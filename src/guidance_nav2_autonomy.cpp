/*
guidance_nav2_autonomy.cpp ===========================================

* Author: Autumn Peterson for PAVBot Capstone Team, 2026
* Purpose: Converts the generated guidance path into continuously updated
           Nav2 NavigateToPose goals. This node acts as the autonomy bridge
           between the local guidance path and the Nav2 navigation stack.

* Subscribes to:
  - /guidance/path (nav_msgs/msg/Path)
      Local guidance path produced by guidance_path_builder

* Provides Service:
  - /autonomy/set_enabled (std_srvs/srv/SetBool)
      Enables or disables autonomous goal streaming

* Sends Action Goals To:
  - /navigate_to_pose (nav2_msgs/action/NavigateToPose)
      Nav2 action server used to move the robot toward the selected goal

* Notes:
  - Selects a lookahead pose along the guidance path.
  - Transforms the selected goal into the Nav2 global frame, usually odom.
  - Continuously streams updated Nav2 goals while autonomy is enabled.
  - Uses tangent-based orientation so the robot faces along the path.
  - Smooths yaw to reduce jitter from noisy path estimates.
  - Rejects stale, short, or redundant paths/goals.
=========================================================================
*/

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

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"


using namespace std::chrono_literals;

class GuidanceNav2Autonomy : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  GuidanceNav2Autonomy(): Node("guidance_nav2_autonomy"), tf_buffer_(this->get_clock()),tf_listener_(tf_buffer_)
    /*
      Purpose: Initializes the Nav2 autonomy bridge, declares and loads
              parameters, subscribes to the guidance path, creates the Nav2
              action client, creates the enable/disable service, and starts
              the periodic goal update timer.

      Input(s):
        * None directly. Uses ROS parameters for topics, frames, lookahead,
          goal update behavior, path freshness, and yaw smoothing.

      Output(s):
        * None. Sets up ROS communication and Nav2 action interfaces.
    */
  {
    // Parameters
    this->declare_parameter<std::string>("guidance_topic", "/guidance/path");
    this->declare_parameter<std::string>("global_frame", "odom");
    this->declare_parameter<std::string>("robot_frame", "base_link");
    this->declare_parameter<double>("lookahead_m", 4.0);
    this->declare_parameter<double>("goal_update_hz", 10.0); //change to make higher
    this->declare_parameter<double>("min_goal_separation_m", 0.2);
    this->declare_parameter<double>("path_stale_sec", 0.25);

    //new smoothing params
    this->declare_parameter<bool>("use_tangent_orientation", true);
    this->declare_parameter<double>("yaw_smooth_alpha", 0.85);     // 0=no smoothing, 0.85 good start
    this->declare_parameter<double>("min_tangent_norm", 1e-3);     // avoid degeneracy

    // new paramter
    this->declare_parameter<int>("min_path_points", 5);
    this->declare_parameter<double>("min_goal_update_sec", 0.2);
    this->declare_parameter<double>("replan_when_dist_to_goal_lt_m", 1.5);
    this->declare_parameter<double>("force_replan_if_goal_moves_m", 4.0);
    this->declare_parameter<double>("goal_progress_timeout_sec", 3.0);


    min_goal_update_sec_ = this->get_parameter("min_goal_update_sec").as_double();
    min_path_points_ = this->get_parameter("min_path_points").as_int();



    guidance_topic_ = this->get_parameter("guidance_topic").as_string();
    global_frame_ = this->get_parameter("global_frame").as_string();
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    lookahead_m_ = this->get_parameter("lookahead_m").as_double();
    goal_update_hz_ = this->get_parameter("goal_update_hz").as_double();
    min_goal_sep_m_ = this->get_parameter("min_goal_separation_m").as_double();
    path_stale_sec_ = this->get_parameter("path_stale_sec").as_double();

    // load new params
    use_tangent_orientation_ = this->get_parameter("use_tangent_orientation").as_bool();
    yaw_smooth_alpha_ = this->get_parameter("yaw_smooth_alpha").as_double();
    min_tangent_norm_ = this->get_parameter("min_tangent_norm").as_double();

    replan_when_dist_to_goal_lt_m_ = this->get_parameter("replan_when_dist_to_goal_lt_m").as_double();
    force_replan_if_goal_moves_m_  = this->get_parameter("force_replan_if_goal_moves_m").as_double();
    goal_progress_timeout_sec_     = this->get_parameter("goal_progress_timeout_sec").as_double();



    // Sub to centerline path
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      guidance_topic_, rclcpp::QoS(10),
      std::bind(&GuidanceNav2Autonomy::onPath, this, std::placeholders::_1));

    // Nav2 action client
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Enable/disable service
    srv_ = this->create_service<std_srvs::srv::SetBool>(
      "/autonomy/set_enabled",
      std::bind(&GuidanceNav2Autonomy::onSetEnabled, this, std::placeholders::_1, std::placeholders::_2));

    // Timer (does nothing unless enabled)
    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, goal_update_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&GuidanceNav2Autonomy::tick, this));

    RCLCPP_INFO(this->get_logger(), "Guidance autonomy node ready (idle). Use /autonomy/set_enabled.");
  }

private:

  static double wrapPi(double a){
    /*
      Purpose: Wraps an angle into the range [-pi, pi] for stable yaw error
              calculations.

      Input(s):
        * double a:
          Input angle in radians.

      Output(s):
        * double:
          Wrapped angle in radians.
    */
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static geometry_msgs::msg::Quaternion quatFromYaw(double yaw){
    /*
      Purpose: Converts a planar yaw angle into a ROS quaternion.

      Input(s):
        * double yaw:
          Desired heading angle in radians.

      Output(s):
        * geometry_msgs::msg::Quaternion:
          Quaternion representing roll = 0, pitch = 0, yaw = input yaw.
    */
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    geometry_msgs::msg::Quaternion out;
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    out.w = q.w();
    return out;
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr msg) {
    /*
      Purpose: Receives the latest guidance path and stores it if it contains
              enough points to be useful for lookahead goal selection.

      Input(s):
        * const nav_msgs::msg::Path::SharedPtr msg:
          Incoming guidance path from guidance_path_builder.

      Output(s):
        * None. Updates the internally stored path and timestamp.
    */
    // Ignore empty / too-short paths so we keep the last usable one briefly
    if ((int)msg->poses.size() < min_path_points_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Ignoring short centerline (N=%zu). Keeping last valid path.", msg->poses.size());
      return;
    }

    std::lock_guard<std::mutex> lk(mutex_);
    last_path_ = *msg;
    last_path_time_ = this->now();
  }

  void onSetEnabled(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    /*
      Purpose: Enables or disables autonomous Nav2 goal streaming. When disabled,
              the current Nav2 goal is cancelled.

      Input(s):
        * const std::shared_ptr<std_srvs::srv::SetBool::Request> req:
          Service request where true enables autonomy and false disables it.

        * std::shared_ptr<std_srvs::srv::SetBool::Response> res:
          Service response containing success status and message.

      Output(s):
        * None. Updates enabled state and may cancel the current goal.
    */

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

  void tick() {
    /*
      Purpose: Periodic autonomy loop that selects a lookahead goal from the
              latest guidance path, transforms it into the Nav2 global frame,
              and sends it to the Nav2 NavigateToPose action server.

      Pipeline:
        1. Return immediately if autonomy is disabled
        2. Verify the Nav2 action server is available
        3. Copy the latest stored guidance path
        4. Reject missing, short, or stale paths
        5. Select a lookahead pose along the path
        6. Transform that pose into the Nav2 global frame
        7. Suppress redundant goal updates
        8. Rate-limit goal sending
        9. Send the updated goal to Nav2

      Input(s):
        * None directly. Uses stored path, TF, Nav2 action client, and parameters.

      Output(s):
        * None. Sends NavigateToPose action goals when appropriate.
    */
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

    // check orientation changes
    if (!path_copy.header.frame_id.empty() && path_copy.header.frame_id != robot_frame_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Centerline frame_id='%s' differs from robot_frame='%s'. This is OK if TF exists from '%s' to '%s'.",
        path_copy.header.frame_id.c_str(), robot_frame_.c_str(),
        path_copy.header.frame_id.c_str(), global_frame_.c_str());
    }


    if (path_copy.poses.size() < static_cast<std::size_t>(min_path_points_)) {
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

    // Transform goal into global_frame for Nav2 using latest available TF
    geometry_msgs::msg::PoseStamped goal_local = *goal_in_path_frame;
    goal_local.header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    geometry_msgs::msg::PoseStamped goal_global;
    try {
      goal_global = tf_buffer_.transform(
        goal_local,
        global_frame_,
        tf2::durationFromSec(0.1));
      goal_global.header.stamp = this->now();
      goal_global.header.frame_id = global_frame_;
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

    // added min goal update period
    const double dt_goal = (this->now() - last_goal_send_time_).seconds();
    if (dt_goal < min_goal_update_sec_) return;

    sendGoal(goal_global);
    last_goal_send_time_ = this->now(); 
    last_goal_global_ = goal_global;
    has_last_goal_ = true;
  }

  std::optional<geometry_msgs::msg::PoseStamped> pickLookaheadPose(const nav_msgs::msg::Path & path, double lookahead_m){
    /*
      Purpose: Selects a goal pose a specified distance ahead along the guidance
              path and assigns an orientation aligned with the local path tangent.

      Input(s):
        * const nav_msgs::msg::Path& path:
          Guidance path to sample.

        * double lookahead_m:
          Desired arc-length distance ahead of the robot for the goal.

      Output(s):
        * std::optional<geometry_msgs::msg::PoseStamped>:
          Selected lookahead pose if the path is valid, otherwise nullopt.
    */

      if (path.poses.size() < 3) return std::nullopt;

      // Walk along arc length to find the lookahead index
      double acc = 0.0;
      size_t chosen = path.poses.size() - 1; // fallback

      for (size_t i = 1; i < path.poses.size(); ++i) {
        const auto & p0 = path.poses[i - 1].pose.position;
        const auto & p1 = path.poses[i].pose.position;
        acc += std::hypot(p1.x - p0.x, p1.y - p0.y);
        if (acc >= lookahead_m) {
          chosen = i;
          break;
        }
      }

      auto goal = path.poses[chosen];
      goal.header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

      if (!use_tangent_orientation_) {
        // leave orientation as is (though lane detector publishes identity)
        return goal;
      }

      // Choose neighbor points to compute tangent direction robustly
      size_t i0, i1;
      if (chosen == 0) {
        i0 = 0; i1 = 1;
      } else if (chosen >= path.poses.size() - 1) {
        i0 = path.poses.size() - 2; i1 = path.poses.size() - 1;
      } else {
        i0 = chosen - 1; i1 = chosen + 1;
      }

      const auto & a = path.poses[i0].pose.position;
      const auto & b = path.poses[i1].pose.position;

      const double dx = b.x - a.x;
      const double dy = b.y - a.y;

      // If tangent is degenerate, point toward the goal in robot frame as fallback
      double yaw = 0.0;
      const double n = std::hypot(dx, dy);

      if (std::isfinite(dx) && std::isfinite(dy) && (n > min_tangent_norm_)) {
        yaw = std::atan2(dy, dx);
      } else {
        const double gx = goal.pose.position.x;
        const double gy = goal.pose.position.y;
        const double gn = std::hypot(gx, gy);
        if (gn > min_tangent_norm_) yaw = std::atan2(gy, gx);
      }

      // Smooth yaw over time to avoid flicker from noisy centerline
      yaw = wrapPi(yaw);
      const double a_s = std::max(0.0, std::min(0.99, yaw_smooth_alpha_));

      if (!has_last_yaw_) {
        last_yaw_ = yaw;
        has_last_yaw_ = true;
      } else {
        const double err = wrapPi(yaw - last_yaw_);
        last_yaw_ = wrapPi(last_yaw_ + (1.0 - a_s) * err);
      }

      goal.pose.orientation = quatFromYaw(last_yaw_);
      return goal;
    }

  void sendGoal(const geometry_msgs::msg::PoseStamped & goal) {
    /*
      Purpose: Sends a NavigateToPose action goal to Nav2 and stores the current
              goal handle when accepted.

      Input(s):
        * const geometry_msgs::msg::PoseStamped& goal:
          Goal pose in the Nav2 global frame.

      Output(s):
        * None. Asynchronously sends a goal to the Nav2 action server.
    */

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

  void cancelCurrentGoal() {
    /*
      Purpose: Cancels the active Nav2 goal if one exists and clears the stored
              goal state.

      Input(s):
        * None.

      Output(s):
        * None. Sends an asynchronous cancel request to Nav2.
    */

    if (current_goal_handle_) {
      nav_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_.reset();
    }
    has_last_goal_ = false;
  }

  // Params
  std::string guidance_topic_;
  std::string global_frame_;
  std::string robot_frame_;
  double lookahead_m_{6.0};
  double goal_update_hz_{2.0};
  double min_goal_sep_m_{1.0};
  double path_stale_sec_{0.5};

  //new params
  int min_path_points_{5};
  rclcpp::Time last_goal_send_time_{0, 0, RCL_ROS_TIME};
  double min_goal_update_sec_{0.2};

  double replan_when_dist_to_goal_lt_m_{1.5};
  double force_replan_if_goal_moves_m_{4.0};
  double goal_progress_timeout_sec_{3.0};

  double last_dist_to_goal_{1e9};
  rclcpp::Time last_progress_time_{0, 0, RCL_ROS_TIME};




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

  // orientation behavior
  bool use_tangent_orientation_{true};
  double yaw_smooth_alpha_{0.85};
  double min_tangent_norm_{1e-3};

  bool has_last_yaw_{false};
  double last_yaw_{0.0};


  bool enabled_{false};
  bool has_last_goal_{false};
  geometry_msgs::msg::PoseStamped last_goal_global_;
};

// MAIN ======================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuidanceNav2Autonomy>());
  rclcpp::shutdown();
  return 0;
}
