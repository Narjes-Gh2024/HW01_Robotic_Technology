from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1) Low-pass filter:  raw IMU -> /imu/filtered
    lowpass_node = Node(
        package='robot_estimation',
        executable='low_pass_filter_node',
        name='low_pass_filter',
        output='screen',
        parameters=[{
            'imu_topic_in':  '/zed/zed_node/imu/data_raw',
            'imu_topic_out': '/imu/filtered',
        }]
    )

    # 2) Bias correction: /imu/filtered -> /imu/bias_corrected
    bias_node = Node(
        package='robot_estimation',
        executable='bias_correct_imu_node',
        name='bias_correct_imu',
        output='screen',
        parameters=[{
            'imu_topic_in':  '/imu/filtered',
            'imu_topic_out': '/imu/bias_corrected',
            'bias_sample_count': 300,
            'recalib_period_sec': 30.0,
        }]
    )

    # 3) Complementary yaw: /imu/bias_corrected -> /estimation/orientation
    complementary_node = Node(
        package='robot_estimation',
        executable='complementary_yaw_node',
        name='complementary_yaw',
        output='screen',
        parameters=[{
            'imu_topic_in':          '/imu/bias_corrected',
            'orientation_topic_out': '/estimation/orientation',
            'alpha': 0.98,
        }]
    )

    # 4) Diff-drive controller: /cmd_vel -> /motor_commands
    diff_drive_ctrl_node = Node(
        package='robot_estimation',
        executable='diff_drive_controller_node',
        name='diff_drive_controller',
        output='screen',
        parameters=[{
            'wheel_radius': 0.1,
            'base_width':   0.5,
        }]
    )

    # 5) Motor command splitter: /motor_commands -> left/right rpm
    motor_cmd_node = Node(
        package='robot_estimation',
        executable='motor_command_node',
        name='motor_command_node',
        output='screen'
    )

    # 6) Motion model odom: /wheel_encoder/odom -> /motion_model/odom + TF
    motion_model_odom_node = Node(
        package='robot_estimation',
        executable='motion_model_odom_node',
        name='motion_model_odom',
        output='screen',
        parameters=[{
            'wheel_odom_topic': '/wheel_encoder/odom',
            'odom_topic':       '/motion_model/odom',
        }]
    )

    return LaunchDescription([
        lowpass_node,
        bias_node,
        complementary_node,
        diff_drive_ctrl_node,
        motor_cmd_node,
        motion_model_odom_node,
    ])
