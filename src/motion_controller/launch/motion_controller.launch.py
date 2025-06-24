from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pose_topic',
            default_value='/robot_pose',
            description='Motion controller robot pose input (should be transformed to be in -1 <= (x,y) <= 1)'
        ),
        DeclareLaunchArgument(
            'controller_freq',
            default_value='30.0',
            description='Controller frequency in Hz'
        ),
        Node(
            package='motion_controller',
            executable='motion_controller',
            name='motion_controller',
            parameters=[{
                'pose_topic': LaunchConfiguration('pose_topic'),
                'controller_freq': LaunchConfiguration('controller_freq')
            }],
            output='screen'
        )
    ])