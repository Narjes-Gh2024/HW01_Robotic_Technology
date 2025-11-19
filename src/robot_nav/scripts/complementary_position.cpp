#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <string>

class ComplementaryPosePathNode : public rclcpp::Node
{
public:
  ComplementaryPosePathNode()
  : Node("complementary_pose_path_node")
  {
    // Input IMU topic (ideally low-pass + bias corrected)
    imu_topic_in_ = declare_parameter<std::string>(
        "imu_topic_in", "/imu/bias_corrected");

    // Orientation output (for debugging / RViz)
    orientation_topic_out_ = declare_parameter<std::string>(
        "orientation_topic_out", "/estimation/orientation");

    // Path output
    path_topic_out_ = declare_parameter<std::string>(
        "path_topic_out", "/estimation/path");

    // Complementary filter coefficient (0..1)
    alpha_ = declare_parameter<double>("alpha", 0.98);

    // Gravity magnitude (m/s^2)
    gravity_ = declare_parameter<double>("gravity", 9.81);

    // Frame used for path & orientation
    frame_id_ = declare_parameter<std::string>("frame_id", "world");

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_in_, rclcpp::SensorDataQoS(),
        std::bind(&ComplementaryPosePathNode::imuCallback, this, std::placeholders::_1));

    orientation_pub_ =
        create_publisher<geometry_msgs::msg::QuaternionStamped>(orientation_topic_out_, 10);

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_out_, 10);

    path_msg_.header.frame_id = frame_id_;

    RCLCPP_INFO(this->get_logger(),
      "ComplementaryPosePathNode started.\n"
      "  imu_in='%s'\n  orientation_out='%s'\n  path_out='%s'\n"
      "  alpha=%.3f, gravity=%.3f, frame_id='%s'",
      imu_topic_in_.c_str(),
      orientation_topic_out_.c_str(),
      path_topic_out_.c_str(),
      alpha_, gravity_, frame_id_.c_str());
  }

private:
  // Parameters / topics
  std::string imu_topic_in_;
  std::string orientation_topic_out_;
  std::string path_topic_out_;
  std::string frame_id_;

  double alpha_;
  double gravity_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr orientation_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // State: orientation (RPY), velocity, position
  double roll_est_{0.0};
  double pitch_est_{0.0};
  double yaw_est_{0.0};

  double vx_{0.0}, vy_{0.0}, vz_{0.0};
  double px_{0.0}, py_{0.0}, pz_{0.0};

  bool first_msg_{true};
  rclcpp::Time last_time_;

  nav_msgs::msg::Path path_msg_;

  static double wrapToPi(double a)
  {
    while (a <= -M_PI) a += 2.0 * M_PI;
    while (a >   M_PI) a -= 2.0 * M_PI;
    return a;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const rclcpp::Time t = msg->header.stamp;

    if (first_msg_) {
      if (t.nanoseconds() == 0) {
        // If no valid stamp, use node clock
        last_time_ = now();
      } else {
        last_time_ = t;
      }

      // Initialize orientation estimate from IMU orientation
      double r0, p0, y0;
      tf2::Quaternion q0(
          msg->orientation.x,
          msg->orientation.y,
          msg->orientation.z,
          msg->orientation.w);
      q0.normalize();
      tf2::Matrix3x3(q0).getRPY(r0, p0, y0);

      roll_est_  = wrapToPi(r0);
      pitch_est_ = wrapToPi(p0);
      yaw_est_   = wrapToPi(y0);

      // Start at origin with zero velocity
      vx_ = vy_ = vz_ = 0.0;
      px_ = py_ = pz_ = 0.0;

      path_msg_.poses.clear();

      first_msg_ = false;
      return;
    }

    double dt = (t - last_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) {
      // protect against bad timestamps; assume small dt
      dt = 1e-3;
    }
    last_time_ = t;

    // 1) Read gyro (rad/s) & accel (m/s^2) in IMU/body frame
    const double wx = msg->angular_velocity.x;
    const double wy = msg->angular_velocity.y;
    const double wz = msg->angular_velocity.z;

    const double ax = msg->linear_acceleration.x;
    const double ay = msg->linear_acceleration.y;
    const double az = msg->linear_acceleration.z;

    // 2) Measured orientation from IMU fusion (roll_meas, pitch_meas, yaw_meas)
    tf2::Quaternion q_meas(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    q_meas.normalize();

    double roll_meas, pitch_meas, yaw_meas;
    tf2::Matrix3x3(q_meas).getRPY(roll_meas, pitch_meas, yaw_meas);
    roll_meas  = wrapToPi(roll_meas);
    pitch_meas = wrapToPi(pitch_meas);
    yaw_meas   = wrapToPi(yaw_meas);

    // 3) Gyro integration
    double roll_gyro  = wrapToPi(roll_est_  + wx * dt);
    double pitch_gyro = wrapToPi(pitch_est_ + wy * dt);
    double yaw_gyro   = wrapToPi(yaw_est_   + wz * dt);

    // 4) Complementary filter for all three angles
    roll_est_  = wrapToPi(alpha_ * roll_gyro  + (1.0 - alpha_) * roll_meas);
    pitch_est_ = wrapToPi(alpha_ * pitch_gyro + (1.0 - alpha_) * pitch_meas);
    yaw_est_   = wrapToPi(alpha_ * yaw_gyro   + (1.0 - alpha_) * yaw_meas);

    // 5) Build orientation quaternion from estimated RPY
    tf2::Quaternion q_est;
    q_est.setRPY(roll_est_, pitch_est_, yaw_est_);
    q_est.normalize();

    // -------- Orientation output (QuaternionStamped) --------
    geometry_msgs::msg::QuaternionStamped q_msg_out;
    q_msg_out.header.stamp = t;
    q_msg_out.header.frame_id = frame_id_;
    q_msg_out.quaternion = tf2::toMsg(q_est);
    orientation_pub_->publish(q_msg_out);

    // -------- Position integration from accel --------
    // Rotate accel from body frame to world frame using q_est
    tf2::Vector3 a_body(ax, ay, az);
    tf2::Vector3 a_world = tf2::quatRotate(q_est, a_body);

    // Subtract gravity in world frame (assuming +Z up)
    a_world.setZ(a_world.z() - gravity_);

    // Integrate acceleration → velocity
    vx_ += a_world.x() * dt;
    vy_ += a_world.y() * dt;
    vz_ += a_world.z() * dt;

    // Integrate velocity → position
    px_ += vx_ * dt;
    py_ += vy_ * dt;
    pz_ += vz_ * dt;

    // -------- Build PoseStamped and append to Path --------
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = t;
    pose.header.frame_id = frame_id_;

    pose.pose.position.x = px_;
    pose.pose.position.y = py_;
    pose.pose.position.z = pz_;
    pose.pose.orientation = q_msg_out.quaternion;

    path_msg_.header.stamp = t;
    path_msg_.header.frame_id = frame_id_;
    path_msg_.poses.push_back(pose);

    // Optionally limit path length to avoid huge arrays
    const std::size_t max_points = 2000;
    if (path_msg_.poses.size() > max_points) {
      path_msg_.poses.erase(path_msg_.poses.begin(),
                            path_msg_.poses.begin() + (path_msg_.poses.size() - max_points));
    }

    path_pub_->publish(path_msg_);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComplementaryPosePathNode>());
  rclcpp::shutdown();
  return 0;
}
