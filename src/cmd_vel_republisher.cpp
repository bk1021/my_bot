#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistToTwistStamped : public rclcpp::Node
{
public:
  TwistToTwistStamped()
  : Node("twist_to_twist_stamped")
  {
    // Publisher
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_stamped", 10);

    // Subscriber
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&TwistToTwistStamped::twist_callback, this, std::placeholders::_1));
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    geometry_msgs::msg::TwistStamped stamped_msg;
    stamped_msg.header.stamp = this->get_clock()->now();
    stamped_msg.header.frame_id = "base_link";  // Set your robot frame
    stamped_msg.twist = *msg;

    pub_->publish(stamped_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistToTwistStamped>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
