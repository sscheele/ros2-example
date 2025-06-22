from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clock_pose_issuer',
            executable='clock_pose_issuer_node',
            name='clock_pose_issuer',
            output='screen',
            parameters=[],
            remappings=[]
        )
    ])