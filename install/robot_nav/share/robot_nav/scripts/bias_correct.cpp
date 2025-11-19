#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <cmath>
#include <string>
#include <chrono>

class BiasCorrectImuMag : public rclcpp::Node
{
public:
  BiasCorrectImuMag()
  : Node("bias_correct_imu_mag")
  {
    // IMU topics
    imu_topic_in_  = declare_parameter<std::string>("imu_topic_in", "/imu/filtered");
    imu_topic_out_ = declare_parameter<std::string>("imu_topic_out", "/imu/bias_corrected");

    // Magnetometer topics
    mag_topic_in_  = declare_parameter<std::string>("mag_topic_in", "/magnetometer/filtered");
    mag_topic_out_ = declare_parameter<std::string>("mag_topic_out", "/magnetometer/bias_corrected");

    // Shared parameters
    bias_sample_count_   = declare_parameter<int>("bias_sample_count", 300);
    recalib_period_sec_  = declare_parameter<double>("recalib_period_sec", 30.0);

    // Subscriptions
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_in_, rclcpp::SensorDataQoS(),
      std::bind(&BiasCorrectImuMag::imuCb, this, std::placeholders::_1));

    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
      mag_topic_in_, rclcpp::SensorDataQoS(),
      std::bind(&BiasCorrectImuMag::magCb, this, std::placeholders::_1));

    // Publishers
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_out_, 10);
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(mag_topic_out_, 10);

    // Timer for recalibration
    timer_ = create_wall_timer(
        std::chrono::duration<double>(recalib_period_sec_),
        std::bind(&BiasCorrectImuMag::resetBiasEstimation, this));

    RCLCPP_INFO(this->get_logger(),
      "BiasCorrectImuMag started.\n"
      "  IMU: in='%s', out='%s'\n"
      "  MAG: in='%s', out='%s'\n"
      "  samples=%d, period=%.1f s",
      imu_topic_in_.c_str(), imu_topic_out_.c_str(),
      mag_topic_in_.c_str(), mag_topic_out_.c_str(),
      bias_sample_count_, recalib_period_sec_);
  }

private:
  // Parameters / topics
  std::string imu_topic_in_, imu_topic_out_;
  std::string mag_topic_in_, mag_topic_out_;
  int    bias_sample_count_;
  double recalib_period_sec_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr  mag_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr               imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr     mag_pub_;
  rclcpp::TimerBase::SharedPtr                                      timer_;

  // IMU bias estimation
  int   imu_current_sample_{0};
  bool  imu_bias_estimated_{false};
  double ax_sum_{0.0}, ay_sum_{0.0}, az_sum_{0.0};
  double gx_sum_{0.0}, gy_sum_{0.0}, gz_sum_{0.0};
  double ax_bias_{0.0}, ay_bias_{0.0}, az_bias_{0.0};
  double gx_bias_{0.0}, gy_bias_{0.0}, gz_bias_{0.0};

  // Magnetometer bias estimation
  int   mag_current_sample_{0};
  bool  mag_bias_estimated_{false};
  double mx_sum_{0.0}, my_sum_{0.0}, mz_sum_{0.0};
  double mx_bias_{0.0}, my_bias_{0.0}, mz_bias_{0.0};

  // Reset both IMU and magnetometer bias estimation
  void resetBiasEstimation()
  {
    RCLCPP_WARN(this->get_logger(),
                "Recalibration triggered: re-estimating IMU & magnetometer bias...");

    // IMU reset
    imu_current_sample_   = 0;
    imu_bias_estimated_   = false;
    ax_sum_ = ay_sum_ = az_sum_ = 0.0;
    gx_sum_ = gy_sum_ = gz_sum_ = 0.0;

    // Magnetometer reset
    mag_current_sample_   = 0;
    mag_bias_estimated_   = false;
    mx_sum_ = my_sum_ = mz_sum_ = 0.0;
  }

  // --------- IMU callback ---------
  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double ax = msg->linear_acceleration.x;
    const double ay = msg->linear_acceleration.y;
    const double az = msg->linear_acceleration.z;

    const double gx = msg->angular_velocity.x;
    const double gy = msg->angular_velocity.y;
    const double gz = msg->angular_velocity.z;

    // 1) Estimation phase
    if (!imu_bias_estimated_) {
      ax_sum_ += ax;
      ay_sum_ += ay;
      az_sum_ += az;

      gx_sum_ += gx;
      gy_sum_ += gy;
      gz_sum_ += gz;

      imu_current_sample_++;

      if (imu_current_sample_ >= bias_sample_count_) {
        ax_bias_ = ax_sum_ / imu_current_sample_;
        ay_bias_ = ay_sum_ / imu_current_sample_;
        az_bias_ = az_sum_ / imu_current_sample_;

        gx_bias_ = gx_sum_ / imu_current_sample_;
        gy_bias_ = gy_sum_ / imu_current_sample_;
        gz_bias_ = gz_sum_ / imu_current_sample_;

        imu_bias_estimated_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "IMU bias updated: accel=(%.4f %.4f %.4f), gyro=(%.4f %.4f %.4f)",
                    ax_bias_, ay_bias_, az_bias_,
                    gx_bias_, gy_bias_, gz_bias_);
      }

      // Pass-through during estimation
      imu_pub_->publish(*msg);
      return;
    }

    // 2) Apply bias correction
    sensor_msgs::msg::Imu out = *msg;

    out.linear_acceleration.x = ax - ax_bias_;
    out.linear_acceleration.y = ay - ay_bias_;
    out.linear_acceleration.z = az - az_bias_;

    out.angular_velocity.x = gx - gx_bias_;
    out.angular_velocity.y = gy - gy_bias_;
    out.angular_velocity.z = gz - gz_bias_;

    imu_pub_->publish(out);
  }

  // --------- Magnetometer callback ---------
  void magCb(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    const double mx = msg->magnetic_field.x;
    const double my = msg->magnetic_field.y;
    const double mz = msg->magnetic_field.z;

    // 1) Estimation phase
    if (!mag_bias_estimated_) {
      mx_sum_ += mx;
      my_sum_ += my;
      mz_sum_ += mz;

      mag_current_sample_++;

      if (mag_current_sample_ >= bias_sample_count_) {
        mx_bias_ = mx_sum_ / mag_current_sample_;
        my_bias_ = my_sum_ / mag_current_sample_;
        mz_bias_ = mz_sum_ / mag_current_sample_;

        mag_bias_estimated_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "MAG bias updated: B=(%.6e %.6e %.6e)",
                    mx_bias_, my_bias_, mz_bias_);
      }

      // Pass-through during estimation
      mag_pub_->publish(*msg);
      return;
    }

    // 2) Apply bias correction
    sensor_msgs::msg::MagneticField out = *msg;

    out.magnetic_field.x = mx - mx_bias_;
    out.magnetic_field.y = my - my_bias_;
    out.magnetic_field.z = mz - mz_bias_;

    mag_pub_->publish(out);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BiasCorrectImuMag>());
  rclcpp::shutdown();
  return 0;
}
