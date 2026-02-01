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

    odom_topic_ = this->get_parameter("odom_topic").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&OdomTFBroadcaster::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Broadcasting TF %s -> %s from %s",
      odom_frame_.c_str(), base_frame_.c_str(), odom_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf;

    // Use odom message time for TF
    tf.header.stamp = this->now();

    // Enforce frame IDs even if bridge leaves them empty / inconsistent
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

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomTFBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
