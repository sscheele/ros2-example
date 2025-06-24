from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch argument for vel_topic
    vel_topic_arg = DeclareLaunchArgument(
        'vel_topic',
        default_value='/turtle1/cmd_vel',
        description='Target topic for robot-specific velocity commands'
    )
    
    # Declare launch argument for input_pose_topic
    input_pose_topic_arg = DeclareLaunchArgument(
        'input_pose_topic',
        default_value='/turtle1/pose',
        description='Source topic for robot-specific pose (input to vel_relay)'
    )
    
    return LaunchDescription([
        vel_topic_arg,
        input_pose_topic_arg,
        
        # Launch motion_controller with transformed pose topic
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('motion_controller'),
                    'launch',
                    'motion_controller.launch.py'
                ])
            ]),
            launch_arguments={
                'pose_topic': '/robot_unit_pose'
            }.items()
        ),
        
        # Launch turtlesim_node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node',
            output='screen'
        ),
        
        # Launch velocity relay node
        Node(
            package='clock_follow_launch',
            executable='vel_relay_node',
            name='vel_relay',
            parameters=[{
                'vel_topic': LaunchConfiguration('vel_topic'),
                'pose_topic': LaunchConfiguration('input_pose_topic')
            }],
            output='screen'
        ),
        
        # Launch clock_pose_issuer
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('clock_pose_issuer'),
                    'launch',
                    'clock_pose_issuer.launch.py'
                ])
            ])
        ),
        
        # Launch gui_pose_issuer
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gui_pose_issuer'),
                    'launch',
                    'gui_pose_issuer.launch.py'
                ])
            ])
        ),
    ])