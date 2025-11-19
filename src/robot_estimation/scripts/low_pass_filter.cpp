#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
#include <string>

class LowPassFilter : public rclcpp::Node {
public:
  LowPassFilter() : Node("low_pass_filter") {
    // IMU topics
    imu_topic_in_  = declare_parameter<std::string>(
        "imu_topic_in", "/zed/zed_node/imu/data_raw");
    imu_topic_out_ = declare_parameter<std::string>(
        "imu_topic_out", "/imu/filtered");

    // --------- Low-pass filter parameters for 3 accel and 3 gyro ----------
    fs_ax_ = declare_parameter<double>("fs_ax", 100.0);
    fs_ay_ = declare_parameter<double>("fs_ay", 100.0);
    fs_az_ = declare_parameter<double>("fs_az", 100.0);
    fs_gx_ = declare_parameter<double>("fs_gx", 100.0);
    fs_gy_ = declare_parameter<double>("fs_gy", 100.0);
    fs_gz_ = declare_parameter<double>("fs_gz", 100.0);

    fc_ax_ = declare_parameter<double>("fc_ax", 10.0);   // e.g. 5–10 Hz for accel
    fc_ay_ = declare_parameter<double>("fc_ay", 10.0);
    fc_az_ = declare_parameter<double>("fc_az", 10.0);
    fc_gx_ = declare_parameter<double>("fc_gx", 20.0);   // e.g. 20–50 Hz for gyro
    fc_gy_ = declare_parameter<double>("fc_gy", 20.0);
    fc_gz_ = declare_parameter<double>("fc_gz", 20.0);

    // Alpha exactly from slide: alpha = 2π fc / (2π fc + fs)
    alpha_ax_ = (2.0 * M_PI * fc_ax_) / (2.0 * M_PI * fc_ax_ + fs_ax_);
    alpha_ay_ = (2.0 * M_PI * fc_ay_) / (2.0 * M_PI * fc_ay_ + fs_ay_);
    alpha_az_ = (2.0 * M_PI * fc_az_) / (2.0 * M_PI * fc_az_ + fs_az_);
    alpha_gx_ = (2.0 * M_PI * fc_gx_) / (2.0 * M_PI * fc_gx_ + fs_gx_);
    alpha_gy_ = (2.0 * M_PI * fc_gy_) / (2.0 * M_PI * fc_gy_ + fs_gy_);
    alpha_gz_ = (2.0 * M_PI * fc_gz_) / (2.0 * M_PI * fc_gz_ + fs_gz_);

    // Subscriber: raw IMU
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_in_, rclcpp::SensorDataQoS(),
      std::bind(&LowPassFilter::imuCb, this, std::placeholders::_1));

    // Publisher: filtered IMU
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_out_, 10);

    RCLCPP_INFO(this->get_logger(), "Lowpass IMU filter node started.");
  }

private:
  std::string imu_topic_in_;
  std::string imu_topic_out_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr    imu_pub_;

  // --------- Low-pass filter state and parameters (6 components) ----------
  double fs_ax_, fs_ay_, fs_az_, fs_gx_, fs_gy_, fs_gz_;
  double fc_ax_, fc_ay_, fc_az_, fc_gx_, fc_gy_, fc_gz_;
  double alpha_ax_, alpha_ay_, alpha_az_, alpha_gx_, alpha_gy_, alpha_gz_;

  double ax_prev_f_{0.0}, ay_prev_f_{0.0}, az_prev_f_{0.0};
  double gx_prev_f_{0.0}, gy_prev_f_{0.0}, gz_prev_f_{0.0};
  bool first_imu_{true};

  // Filtered outputs
  double ax_f_{0.0}, ay_f_{0.0}, az_f_{0.0};
  double gx_f_{0.0}, gy_f_{0.0}, gz_f_{0.0};

  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // 1) Raw accel and gyro
    double ax_raw = msg->linear_acceleration.x;
    double ay_raw = msg->linear_acceleration.y;
    double az_raw = msg->linear_acceleration.z;

    double gx_raw = msg->angular_velocity.x;
    double gy_raw = msg->angular_velocity.y;
    double gz_raw = msg->angular_velocity.z;

    // Initialize y_prev with first sample: y[0] = x[0]
    if (first_imu_) {
      ax_prev_f_ = ax_raw;
      ay_prev_f_ = ay_raw;
      az_prev_f_ = az_raw;
      gx_prev_f_ = gx_raw;
      gy_prev_f_ = gy_raw;
      gz_prev_f_ = gz_raw;
      first_imu_ = false;
    }

    // 2) Low-pass filter (from slide)
    // y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    ax_f_ = alpha_ax_ * ax_raw + (1.0 - alpha_ax_) * ax_prev_f_;
    ay_f_ = alpha_ay_ * ay_raw + (1.0 - alpha_ay_) * ay_prev_f_;
    az_f_ = alpha_az_ * az_raw + (1.0 - alpha_az_) * az_prev_f_;

    gx_f_ = alpha_gx_ * gx_raw + (1.0 - alpha_gx_) * gx_prev_f_;
    gy_f_ = alpha_gy_ * gy_raw + (1.0 - alpha_gy_) * gy_prev_f_;
    gz_f_ = alpha_gz_ * gz_raw + (1.0 - alpha_gz_) * gz_prev_f_;

    // Update previous y[n-1]
    ax_prev_f_ = ax_f_;
    ay_prev_f_ = ay_f_;
    az_prev_f_ = az_f_;

    gx_prev_f_ = gx_f_;
    gy_prev_f_ = gy_f_;
    gz_prev_f_ = gz_f_;

    // 3) Build filtered IMU message and publish
    sensor_msgs::msg::Imu out = *msg;  // copy everything (orientation, covariances, header, ...)
    out.linear_acceleration.x = ax_f_;
    out.linear_acceleration.y = ay_f_;
    out.linear_acceleration.z = az_f_;

    out.angular_velocity.x = gx_f_;
    out.angular_velocity.y = gy_f_;
    out.angular_velocity.z = gz_f_;

    imu_pub_->publish(out);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LowPassFilter>());
  rclcpp::shutdown();
  return 0;
}

