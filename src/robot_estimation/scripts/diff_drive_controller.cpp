#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>

class DiffDriveController : public rclcpp::Node
{
public:
    DiffDriveController() : Node("diff_drive_controller")
    {
        wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.1);    
        base_width_   = this->declare_parameter<double>("base_width",   0.5);     


        motor_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/motor_commands", 10);


        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&DiffDriveController::cmdVelCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "DiffDriveController Node started.");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {


        double v     = msg->linear.x;
        double omega = msg->angular.z;

 
        double v_r = v + omega * base_width_ / 2.0;
        double v_l = v - omega * base_width_ / 2.0;


        double w_r = v_r / wheel_radius_;
        double w_l = v_l / wheel_radius_;


        double rpm_r = w_r * 60.0 / (2.0 * M_PI);
        double rpm_l = w_l * 60.0 / (2.0 * M_PI);


        std_msgs::msg::Float64MultiArray motor_cmd;
        motor_cmd.data.resize(2);
        motor_cmd.data[0] = rpm_l;  
        motor_cmd.data[1] = rpm_r;  

        motor_cmd_pub_->publish(motor_cmd);
    }

    double wheel_radius_;
    double base_width_;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_cmd_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiffDriveController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

