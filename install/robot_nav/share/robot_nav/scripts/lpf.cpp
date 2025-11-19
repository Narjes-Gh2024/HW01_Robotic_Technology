#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <string>
#include <cmath>

class LowPassFilterNode : public rclcpp::Node
{
public:
  LowPassFilterNode()
  : Node("low_pass_filter_node"),
    initialized_(false),
    mag_initialized_(false)
  {
    double sf = 10.0;    // sampling freq
    double df = 5.0;     // damping freq
    double di = 3.0;     // damping intensity

    double attenuation = std::pow(10.0, di / -10.0);
    alpha_ = std::exp( -(2.0 * M_PI * df) / (sf * attenuation) );

    // IMU topics (accel + gyro)
    input_topic_  = this->declare_parameter<std::string>("input_topic", "/phone/imu");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/imu/filtered");

    // Magnetometer topics
    mag_input_topic_ = this->declare_parameter<std::string>("mag_input_topic", "/phone/magnetometer");
    mag_output_topic_ = this->declare_parameter<std::string>("mag_output_topic", "/magnetometer/filtered");

    RCLCPP_INFO(
      this->get_logger(),
      "LowPassFilterNode started. sf=%.1f Hz, df=%.1f Hz, alpha=%.5f\n"
      "  IMU   : input='%s', output='%s'\n"
      "  MAG   : input='%s', output='%s'",
      sf, df, alpha_,
      input_topic_.c_str(), output_topic_.c_str(),
      mag_input_topic_.c_str(), mag_output_topic_.c_str());

    // IMU (accel + gyro)
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      input_topic_, 50,
      std::bind(&LowPassFilterNode::imuCallback, this, std::placeholders::_1));
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_topic_, 10);

    // Magnetometer
    mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      mag_input_topic_, 50,
      std::bind(&LowPassFilterNode::magCallback, this, std::placeholders::_1));
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>(mag_output_topic_, 10);
  }

private:
  // ---------- IMU: accel + gyro ----------
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!initialized_) {
      prev_acc_x_  = msg->linear_acceleration.x;
      prev_acc_y_  = msg->linear_acceleration.y;
      prev_acc_z_  = msg->linear_acceleration.z;
      prev_gyro_x_ = msg->angular_velocity.x;
      prev_gyro_y_ = msg->angular_velocity.y;
      prev_gyro_z_ = msg->angular_velocity.z;
      initialized_ = true;
    }

    sensor_msgs::msg::Imu out = *msg;

    out.linear_acceleration.x = lowpass(msg->linear_acceleration.x, prev_acc_x_);
    out.linear_acceleration.y = lowpass(msg->linear_acceleration.y, prev_acc_y_);
    out.linear_acceleration.z = lowpass(msg->linear_acceleration.z, prev_acc_z_);

    out.angular_velocity.x = lowpass(msg->angular_velocity.x, prev_gyro_x_);
    out.angular_velocity.y = lowpass(msg->angular_velocity.y, prev_gyro_y_);
    out.angular_velocity.z = lowpass(msg->angular_velocity.z, prev_gyro_z_);

    imu_pub_->publish(out);
  }

  // ---------- Magnetometer ----------
  void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    if (!mag_initialized_) {
      prev_mag_x_ = msg->magnetic_field.x;
      prev_mag_y_ = msg->magnetic_field.y;
      prev_mag_z_ = msg->magnetic_field.z;
      mag_initialized_ = true;
    }

    sensor_msgs::msg::MagneticField out = *msg;

    out.magnetic_field.x = lowpass(msg->magnetic_field.x, prev_mag_x_);
    out.magnetic_field.y = lowpass(msg->magnetic_field.y, prev_mag_y_);
    out.magnetic_field.z = lowpass(msg->magnetic_field.z, prev_mag_z_);

    mag_pub_->publish(out);
  }

  // ---------- Shared low-pass ----------
  double lowpass(double x, double &y)
  {
    y = alpha_ * y + (1.0 - alpha_) * x;
    return y;
  }

  // Filter coefficient
  double alpha_;

  // IMU topics
  std::string input_topic_;
  std::string output_topic_;

  // Magnetometer topics
  std::string mag_input_topic_;
  std::string mag_output_topic_;

  // IMU state
  bool   initialized_;
  double prev_acc_x_, prev_acc_y_, prev_acc_z_;
  double prev_gyro_x_, prev_gyro_y_, prev_gyro_z_;

  // Magnetometer state
  bool   mag_initialized_;
  double prev_mag_x_, prev_mag_y_, prev_mag_z_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr    imu_pub_;

  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr    mag_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LowPassFilterNode>());
  rclcpp::shutdown();
  return 0;
}
