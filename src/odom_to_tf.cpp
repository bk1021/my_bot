#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdomToTF : public rclcpp::Node
{
public:
  OdomToTF()
  : Node("odom_to_tf")
  {
    // Create TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to /odom
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&OdomToTF::odom_callback, this, std::placeholders::_1));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = msg->header.stamp;
    t.header.frame_id = "/odom";          
    t.child_frame_id = "/base_link";

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    t.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTF>());
  rclcpp::shutdown();
  return 0;
}
