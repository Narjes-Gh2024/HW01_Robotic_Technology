#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
#include <string>
#include <chrono>


class BiasCorrectImu : public rclcpp::Node {
public:
  BiasCorrectImu() : Node("bias_correct_imu")
  {
    imu_topic_in_  = declare_parameter<std::string>("imu_topic_in", "/imu/filtered");
    imu_topic_out_ = declare_parameter<std::string>("imu_topic_out", "/imu/bias_corrected");

    bias_sample_count_ = declare_parameter<int>("bias_sample_count", 300);
    recalib_period_sec_ = declare_parameter<double>("recalib_period_sec", 30.0);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_in_, rclcpp::SensorDataQoS(),
      std::bind(&BiasCorrectImu::imuCb, this, std::placeholders::_1));

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_out_, 10);

    // Timer for recalibration every X seconds
    timer_ = create_wall_timer(
        std::chrono::duration<double>(recalib_period_sec_),
        std::bind(&BiasCorrectImu::resetBiasEstimation, this));

    RCLCPP_INFO(this->get_logger(), "BiasCorrectImu started.");
  }

private:
  std::string imu_topic_in_, imu_topic_out_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int bias_sample_count_;
  double recalib_period_sec_;

  // Bias Estimation
  int current_sample_{0};
  bool bias_estimated_{false};

  double ax_sum_{0.0}, ay_sum_{0.0}, az_sum_{0.0};
  double gx_sum_{0.0}, gy_sum_{0.0}, gz_sum_{0.0};

  double ax_bias_{0.0}, ay_bias_{0.0}, az_bias_{0.0};
  double gx_bias_{0.0}, gy_bias_{0.0}, gz_bias_{0.0};

  // Reset every 30 seconds
  void resetBiasEstimation()
  {
    RCLCPP_WARN(this->get_logger(),
                "Recalibration triggered: re-estimating IMU bias...");

    current_sample_ = 0;
    bias_estimated_ = false;

    ax_sum_ = ay_sum_ = az_sum_ = 0.0;
    gx_sum_ = gy_sum_ = gz_sum_ = 0.0;
  }

  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;

    double gx = msg->angular_velocity.x;
    double gy = msg->angular_velocity.y;
    double gz = msg->angular_velocity.z;

    // ---- 1) Bias estimation phase ----
    if (!bias_estimated_) {

      ax_sum_ += ax;
      ay_sum_ += ay;
      az_sum_ += az;

      gx_sum_ += gx;
      gy_sum_ += gy;
      gz_sum_ += gz;

      current_sample_++;

      if (current_sample_ >= bias_sample_count_) {
        // Compute mean bias
        ax_bias_ = ax_sum_ / current_sample_;
        ay_bias_ = ay_sum_ / current_sample_;
        az_bias_ = az_sum_ / current_sample_;

        gx_bias_ = gx_sum_ / current_sample_;
        gy_bias_ = gy_sum_ / current_sample_;
        gz_bias_ = gz_sum_ / current_sample_;

        bias_estimated_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "Bias updated: accel=(%.4f %.4f %.4f), gyro=(%.4f %.4f %.4f)",
                    ax_bias_, ay_bias_, az_bias_,
                    gx_bias_, gy_bias_, gz_bias_);
      }

      imu_pub_->publish(*msg);
      return;
    }

    // ---- 2) Apply bias correction ----
    sensor_msgs::msg::Imu out = *msg;

    out.linear_acceleration.x = ax - ax_bias_;
    out.linear_acceleration.y = ay - ay_bias_;
    out.linear_acceleration.z = az - az_bias_;

    out.angular_velocity.x = gx - gx_bias_;
    out.angular_velocity.y = gy - gy_bias_;
    out.angular_velocity.z = gz - gz_bias_;

    imu_pub_->publish(out);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BiasCorrectImu>());
  rclcpp::shutdown();
  return 0;
}

