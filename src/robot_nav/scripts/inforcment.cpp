#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "httplib.h"   // from robot_nav/include/httplib.h
#include "json.hpp"    // from robot_nav/include/json.hpp

#include <thread>
#include <memory>
#include <string>
#include <functional>

using json = nlohmann::json;

class HttpImuNode : public rclcpp::Node
{
public:
  HttpImuNode()
  : Node("http_imu_node")
  {
    // Publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("phone/imu", 10);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("phone/magnetometer", 10);

    // Parameters
    int port = this->declare_parameter<int>("port", 8000);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "phone_imu");

    RCLCPP_INFO(this->get_logger(),
                "Starting HTTP IMU server on 0.0.0.0:%d at route /data", port);

    server_ = std::make_shared<httplib::Server>();

    // HTTP POST /data handler
    server_->Post("/data",
      [this](const httplib::Request &req, httplib::Response &res)
      {
        try {
          // Parse JSON body
          json j = json::parse(req.body);

          sensor_msgs::msg::Imu imu_msg;
          imu_msg.header.stamp = this->get_clock()->now();
          imu_msg.header.frame_id = frame_id_;

          sensor_msgs::msg::MagneticField mag_msg;
          mag_msg.header = imu_msg.header;  // same timestamp & frame

          bool has_gyro = false;
          bool has_accel = false;
          bool has_mag = false;

          // Recursive visitor to traverse arbitrary JSON structure
          std::function<void(const json&)> visit;
          visit = [&](const json &node)
          {
            if (node.is_object()) {
              if (node.contains("name") && node.contains("values") && node["values"].is_object()) {
                std::string name = node["name"].get<std::string>();
                const auto &values = node["values"];

                // Gyroscope
                if (name == "gyroscope") {
                  imu_msg.angular_velocity.x = values.value("x", 0.0);
                  imu_msg.angular_velocity.y = values.value("y", 0.0);
                  imu_msg.angular_velocity.z = values.value("z", 0.0);
                  has_gyro = true;
                }
                // Accelerometer / Total acceleration
                else if (name == "accelerometer" || name == "totalacceleration") {
                  imu_msg.linear_acceleration.x = values.value("x", 0.0);
                  imu_msg.linear_acceleration.y = values.value("y", 0.0);
                  imu_msg.linear_acceleration.z = values.value("z", 0.0);
                  has_accel = true;
                }
                // Magnetometer (raw x, y, z)
                else if (name == "magnetometer") {
                  // Assuming phone sends microtesla (µT) like Android Sensor API
                  double mx_uT = values.value("x", 0.0);
                  double my_uT = values.value("y", 0.0);
                  double mz_uT = values.value("z", 0.0);

                  // Convert µT -> Tesla for sensor_msgs/MagneticField
                  mag_msg.magnetic_field.x = mx_uT * 1e-6;
                  mag_msg.magnetic_field.y = my_uT * 1e-6;
                  mag_msg.magnetic_field.z = mz_uT * 1e-6;

                  has_mag = true;
                }
              }

              // Visit children of object
              for (const auto &item : node.items()) {
                visit(item.value());
              }
            }
            else if (node.is_array()) {
              for (const auto &el : node) {
                visit(el);
              }
            }
          };

          // Start recursive traversal
          visit(j);

          // Publish IMU if we got gyro or accel
          if (has_gyro || has_accel) {
            imu_pub_->publish(imu_msg);
            RCLCPP_INFO(this->get_logger(),
              "Published IMU: gyro[%.6f %.6f %.6f], accel[%.6f %.6f %.6f]",
              imu_msg.angular_velocity.x,
              imu_msg.angular_velocity.y,
              imu_msg.angular_velocity.z,
              imu_msg.linear_acceleration.x,
              imu_msg.linear_acceleration.y,
              imu_msg.linear_acceleration.z);
          }

          // Publish magnetometer if available
          if (has_mag) {
            mag_msg.header.stamp = this->get_clock()->now();
            mag_pub_->publish(mag_msg);
            RCLCPP_INFO(this->get_logger(),
              "Published magnetometer: B[%.6e %.6e %.6e] Tesla",
              mag_msg.magnetic_field.x,
              mag_msg.magnetic_field.y,
              mag_msg.magnetic_field.z);
          }

          if (!has_gyro && !has_accel && !has_mag) {
            RCLCPP_WARN(this->get_logger(),
              "Received JSON but no gyroscope/accelerometer/magnetometer data found.");
          }

          // HTTP response
          res.set_content("{\"status\":\"success\"}", "application/json");
          res.status = 200;
        }
        catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", e.what());
          res.set_content("{\"status\":\"error\",\"reason\":\"invalid_json\"}",
                          "application/json");
          res.status = 400;
        }
      }
    );

    // Run HTTP server in separate thread so ROS2 spin() is not blocked
    server_thread_ = std::thread([this, port]() {
      server_->listen("0.0.0.0", port);
    });
  }

  ~HttpImuNode() override
  {
    if (server_) {
      server_->stop();
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "HTTP IMU server stopped.");
  }

private:
  std::shared_ptr<httplib::Server> server_;
  std::thread server_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  std::string frame_id_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HttpImuNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
