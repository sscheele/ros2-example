"""Utility functions for converting clock positions to ROS poses."""

import math

from geometry_msgs.msg import Pose


class PoseCalculator:
    """Handles coordinate conversion from clock positions to ROS poses."""

    @staticmethod
    def clock_position_to_pose(click_x: float, click_y: float,
                               center_x: float, center_y: float,
                               radius: float) -> tuple[Pose, tuple[float, float]]:
        """
        Convert a click position on the clock to a ROS Pose message.

        Args:
            click_x, click_y: Pixel coordinates of the click
            center_x, center_y: Pixel coordinates of the clock center
            radius: Radius of the clock in pixels

        Returns:
            Tuple of (geometry_msgs/Pose message, clipped_pixel_coords)
            where clipped_pixel_coords are the pixel coordinates on the unit circle
        """
        # Calculate relative position from center
        rel_x = click_x - center_x
        rel_y = click_y - center_y

        # Calculate distance from center
        distance = math.sqrt(rel_x**2 + rel_y**2)

        # Always normalize to unit circle (clip to circle edge)
        if distance > 0:
            # Normalize to unit circle
            unit_rel_x = rel_x / distance
            unit_rel_y = rel_y / distance
        else:
            # Handle center click - default to 12 o'clock
            unit_rel_x = 0.0
            unit_rel_y = -1.0

        # Calculate clipped pixel coordinates (on the circle edge)
        clipped_x = center_x + unit_rel_x * radius
        clipped_y = center_y + unit_rel_y * radius

        # Convert to unit circle coordinates for ROS
        # Note: Canvas Y coordinates are inverted (0 at top)
        # Clock coordinates: 12 o'clock = (0, 1), 3 o'clock = (1, 0)
        pose_x = unit_rel_x
        pose_y = -unit_rel_y  # Invert Y to match standard coordinates

        # Create pose message
        pose = Pose()
        pose.position.x = pose_x
        pose.position.y = pose_y
        pose.position.z = 0.0

        # Set orientation (facing forward, no rotation)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        return pose, (clipped_x, clipped_y)

    @staticmethod
    def angle_to_clock_position(angle_radians: float) -> tuple:
        """
        Convert an angle to clock position coordinates.

        Args:
            angle_radians: Angle in radians (0 = 3 o'clock, π/2 = 12 o'clock)

        Returns:
            Tuple of (x, y) coordinates on unit circle
        """
        x = math.cos(angle_radians)
        y = math.sin(angle_radians)
        return (x, y)

    @staticmethod
    def is_click_in_circle(click_x: float, click_y: float,
                           center_x: float, center_y: float,
                           radius: float) -> bool:
        """
        Check if a click position is within the clock circle.

        Args:
            click_x, click_y: Pixel coordinates of the click
            center_x, center_y: Pixel coordinates of the clock center
            radius: Radius of the clock in pixels

        Returns:
            True if click is within the circle
        """
        distance = math.sqrt((click_x - center_x)**2 + (click_y - center_y)**2)
        return distance <= radius