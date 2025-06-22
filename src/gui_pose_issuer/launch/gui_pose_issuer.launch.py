"""
Launch file for the GUI pose issuer node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for GUI pose issuer."""
    
    gui_pose_issuer_node = Node(
        package='gui_pose_issuer',
        executable='gui_pose_issuer_node',
        name='gui_pose_issuer',
        output='screen',
        parameters=[],
        remappings=[
            # Topics are already correctly named in the node
        ]
    )
    
    return LaunchDescription([
        gui_pose_issuer_node
    ])