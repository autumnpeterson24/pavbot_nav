#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdomTFBroadcaster : public rclcpp::Node
{
public:
  OdomTFBroadcaster()
  : Node("odom_tf_broadcaster")
  {
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<bool>("bootstrap_tf", true);
    this->declare_parameter<double>("bootstrap_rate_hz", 10.0);

    odom_topic_ = this->get_parameter("odom_topic").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    bootstrap_tf_ = this->get_parameter("bootstrap_tf").as_bool();
    bootstrap_rate_hz_ = this->get_parameter("bootstrap_rate_hz").as_double();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&OdomTFBroadcaster::odomCallback, this, std::placeholders::_1));

    // Bootstrap: publish identity odom->base_link until first odom arrives
    if (bootstrap_tf_ && bootstrap_rate_hz_ > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / bootstrap_rate_hz_);
      bootstrap_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() {
          if (have_odom_) {
            bootstrap_timer_.reset(); // stop timer
            return;
          }
          publishIdentityTf(this->now());
        });
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Broadcasting TF %s -> %s from %s (bootstrap_tf=%s)",
      odom_frame_.c_str(), base_frame_.c_str(), odom_topic_.c_str(),
      bootstrap_tf_ ? "true" : "false");
  }

private:
  void publishIdentityTf(const rclcpp::Time& stamp)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    have_odom_ = true;

    geometry_msgs::msg::TransformStamped tf;

    // use odom message time for TF (correct for /use_sim_time pipelines)
    tf.header.stamp = msg->header.stamp;
    if (tf.header.stamp.sec == 0 && tf.header.stamp.nanosec == 0) {
      // fallback if the bridge gives you a zero stamp
      tf.header.stamp = this->now();
    }

    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;

    tf.transform.translation.x = msg->pose.pose.position.x;
    tf.transform.translation.y = msg->pose.pose.position.y;
    tf.transform.translation.z = msg->pose.pose.position.z;
    tf.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf);
  }

  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  bool bootstrap_tf_{true};
  double bootstrap_rate_hz_{10.0};
  bool have_odom_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr bootstrap_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
