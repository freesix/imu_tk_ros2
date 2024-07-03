from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    log_level = 'info'

    return LaunchDescription([
        DeclareLaunchArgument(
            'ros2_bag_path',
            description="Input bag Path" 
            ),
        DeclareLaunchArgument(
            'imu_topic',
            description="IMU Topic Name"
            ),
                
        Node(
            package="imu_tk_ros2",
            executable="imu_tk_node",
            name="imu_tk_node",
            parameters=[{'ros2_bag_path':LaunchConfiguration('ros2_bag_path')},
                        {'imu_topic':LaunchConfiguration('imu_topic')}
                       ], 
            ros_arguments=['--log-level', log_level]
            )
        ])