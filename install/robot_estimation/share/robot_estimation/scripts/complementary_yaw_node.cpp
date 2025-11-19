#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <string>

class ComplementaryYawNode : public rclcpp::Node {
public:
  ComplementaryYawNode() : Node("complementary_yaw_node")
  {
    // Input IMU topic (ideally bias-corrected and/or low-pass filtered)
    imu_topic_in_ = declare_parameter<std::string>(
        "imu_topic_in", "/imu/bias_corrected");

    // Output orientation topic
    orientation_topic_out_ = declare_parameter<std::string>(
        "orientation_topic_out", "/estimation/orientation");

    // Complementary filter coefficient (0..1), e.g. 0.98
    alpha_ = declare_parameter<double>("alpha", 0.98);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_in_, rclcpp::SensorDataQoS(),
        std::bind(&ComplementaryYawNode::imuCallback, this, std::placeholders::_1));

    orientation_pub_ =
        create_publisher<geometry_msgs::msg::QuaternionStamped>(orientation_topic_out_, 10);

    RCLCPP_INFO(this->get_logger(), "ComplementaryYawNode (yaw only) started.");
  }

private:
  std::string imu_topic_in_;
  std::string orientation_topic_out_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr orientation_pub_;

  // Complementary filter parameter
  double alpha_;

  // Estimated yaw state
  double yaw_est_{0.0};
  rclcpp::Time last_time_;
  bool first_msg_{true};

  static double wrapToPi(double a)
  {
    while (a <= -M_PI) a += 2.0 * M_PI;
    while (a >   M_PI) a -= 2.0 * M_PI;
    return a;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Time delta
    rclcpp::Time t = msg->header.stamp;
    if (first_msg_) {
      last_time_ = t;
      // Initialize yaw_est_ from IMU orientation
      double roll, pitch, yaw_meas;
      tf2::Quaternion q_init(
          msg->orientation.x,
          msg->orientation.y,
          msg->orientation.z,
          msg->orientation.w);
      q_init.normalize();
      tf2::Matrix3x3(q_init).getRPY(roll, pitch, yaw_meas);
      yaw_est_ = wrapToPi(yaw_meas);
      first_msg_ = false;
      return;
    }

    double dt = (t - last_time_).seconds();
    if (dt <= 0.0) {
      dt = 1e-3;  // small fallback
    }
    last_time_ = t;

    // 1) Gyro yaw rate (rad/s)
    double gz = msg->angular_velocity.z;

    // 2) Measured yaw from IMU orientation (e.g. from internal fusion / magnetometer)
    const auto &q_msg = msg->orientation;
    tf2::Quaternion q_meas(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    q_meas.normalize();
    double roll_meas, pitch_meas, yaw_meas;
    tf2::Matrix3x3(q_meas).getRPY(roll_meas, pitch_meas, yaw_meas);
    yaw_meas = wrapToPi(yaw_meas);

    // 3) Gyro integration (prediction)
    double yaw_gyro = wrapToPi(yaw_est_ + gz * dt);

    // 4) Complementary filter fusion
    //    yaw_est = alpha * yaw_gyro + (1 - alpha) * yaw_meas
    yaw_est_ = wrapToPi(alpha_ * yaw_gyro + (1.0 - alpha_) * yaw_meas);

    // 5) Build quaternion from yaw only (roll = pitch = 0)
    tf2::Quaternion q_out;
    q_out.setRPY(0.0, 0.0, yaw_est_);
    q_out.normalize();

    geometry_msgs::msg::QuaternionStamped q_msg_out;
    q_msg_out.header.stamp = t;
    q_msg_out.header.frame_id = "base_link";  // or "imu_link", according to your frames
    q_msg_out.quaternion.x = q_out.x();
    q_msg_out.quaternion.y = q_out.y();
    q_msg_out.quaternion.z = q_out.z();
    q_msg_out.quaternion.w = q_out.w();

    // 6) Publish estimated yaw as orientation
    orientation_pub_->publish(q_msg_out);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComplementaryYawNode>());
  rclcpp::shutdown();
  return 0;
}

