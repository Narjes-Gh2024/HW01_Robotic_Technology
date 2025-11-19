#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

class MotionModelOdom : public rclcpp::Node {
public:
  MotionModelOdom() : Node("motion_model_odom")
  {
    // Sub to wheel odometry (velocity inputs)
    wheel_odom_topic_ = declare_parameter<std::string>(
        "wheel_odom_topic", "/wheel_encoder/odom");

    odom_topic_ = declare_parameter<std::string>(
        "odom_topic", "/motion_model/odom");

    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        wheel_odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&MotionModelOdom::wheelOdomCb, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    x_ = 0.0;
    y_ = 0.0;
    th_ = 0.0;
  }

private:
  std::string wheel_odom_topic_, odom_topic_;
  double x_, y_, th_;
  rclcpp::Time last_time_{0,0,RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void wheelOdomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double v = msg->twist.twist.linear.x;
    double omega = msg->twist.twist.angular.z;
    rclcpp::Time t = msg->header.stamp;

    if (!last_time_.nanoseconds())
      last_time_ = t;

    double dt = (t - last_time_).seconds();
    if (dt <= 0) return;

    // MOTION MODEL (DIFF-DRIVE)
    x_ += v * std::cos(th_) * dt;
    y_ += v * std::sin(th_) * dt;
    th_ += omega * dt;

    // PUBLISH ODOM
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, th_);
    q.normalize();

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom_pub_->publish(odom);

    // BROADCAST TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = t;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);

    last_time_ = t;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionModelOdom>());
  rclcpp::shutdown();
  return 0;
}

