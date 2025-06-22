"""
Tests for the pose calculator utility.
"""

import unittest
import math
from gui_pose_issuer.pose_calculator import PoseCalculator


class TestPoseCalculator(unittest.TestCase):
    """Test cases for PoseCalculator class."""
    
    def test_clock_position_to_pose_center(self):
        """Test conversion of center click to pose."""
        pose, (clipped_x, clipped_y) = PoseCalculator.clock_position_to_pose(
            click_x=100, click_y=100,  # Center of 200x200 canvas
            center_x=100, center_y=100,
            radius=50
        )
        
        # Center click should default to 12 o'clock position (0, 1)
        self.assertAlmostEqual(pose.position.x, 0.0, places=3)
        self.assertAlmostEqual(pose.position.y, 1.0, places=3)
        self.assertAlmostEqual(pose.position.z, 0.0, places=3)
        
        # Orientation should be identity quaternion
        self.assertAlmostEqual(pose.orientation.w, 1.0, places=3)
        self.assertAlmostEqual(pose.orientation.x, 0.0, places=3)
        self.assertAlmostEqual(pose.orientation.y, 0.0, places=3)
        self.assertAlmostEqual(pose.orientation.z, 0.0, places=3)
        
        # Clipped position should be at 12 o'clock on circle edge
        self.assertAlmostEqual(clipped_x, 100.0, places=3)  # center_x
        self.assertAlmostEqual(clipped_y, 50.0, places=3)   # center_y - radius
    
    def test_clock_position_to_pose_twelve_oclock(self):
        """Test conversion of 12 o'clock position."""
        pose, (clipped_x, clipped_y) = PoseCalculator.clock_position_to_pose(
            click_x=100, click_y=50,   # Top of circle
            center_x=100, center_y=100,
            radius=50
        )
        
        # 12 o'clock should be (0, 1)
        self.assertAlmostEqual(pose.position.x, 0.0, places=3)
        self.assertAlmostEqual(pose.position.y, 1.0, places=3)
    
    def test_clock_position_to_pose_three_oclock(self):
        """Test conversion of 3 o'clock position."""
        pose, (clipped_x, clipped_y) = PoseCalculator.clock_position_to_pose(
            click_x=150, click_y=100,  # Right of circle
            center_x=100, center_y=100,
            radius=50
        )
        
        # 3 o'clock should be (1, 0)
        self.assertAlmostEqual(pose.position.x, 1.0, places=3)
        self.assertAlmostEqual(pose.position.y, 0.0, places=3)
    
    def test_clock_position_to_pose_six_oclock(self):
        """Test conversion of 6 o'clock position."""
        pose, (clipped_x, clipped_y) = PoseCalculator.clock_position_to_pose(
            click_x=100, click_y=150,  # Bottom of circle
            center_x=100, center_y=100,
            radius=50
        )
        
        # 6 o'clock should be (0, -1)
        self.assertAlmostEqual(pose.position.x, 0.0, places=3)
        self.assertAlmostEqual(pose.position.y, -1.0, places=3)
    
    def test_clock_position_to_pose_nine_oclock(self):
        """Test conversion of 9 o'clock position."""
        pose, (clipped_x, clipped_y) = PoseCalculator.clock_position_to_pose(
            click_x=50, click_y=100,   # Left of circle
            center_x=100, center_y=100,
            radius=50
        )
        
        # 9 o'clock should be (-1, 0)
        self.assertAlmostEqual(pose.position.x, -1.0, places=3)
        self.assertAlmostEqual(pose.position.y, 0.0, places=3)
    
    def test_click_outside_circle_projection(self):
        """Test that clicks outside circle are projected to edge."""
        pose, (clipped_x, clipped_y) = PoseCalculator.clock_position_to_pose(
            click_x=200, click_y=100,  # Far right, outside circle
            center_x=100, center_y=100,
            radius=50
        )
        
        # Should be projected to 3 o'clock position
        self.assertAlmostEqual(pose.position.x, 1.0, places=3)
        self.assertAlmostEqual(pose.position.y, 0.0, places=3)
        
        # Clipped position should be on circle edge at 3 o'clock
        self.assertAlmostEqual(clipped_x, 150.0, places=3)  # center_x + radius
        self.assertAlmostEqual(clipped_y, 100.0, places=3)  # center_y
    
    def test_is_click_in_circle(self):
        """Test circle boundary detection."""
        # Click inside circle
        self.assertTrue(PoseCalculator.is_click_in_circle(
            110, 110, 100, 100, 50
        ))
        
        # Click on circle edge
        self.assertTrue(PoseCalculator.is_click_in_circle(
            150, 100, 100, 100, 50
        ))
        
        # Click outside circle
        self.assertFalse(PoseCalculator.is_click_in_circle(
            200, 100, 100, 100, 50
        ))
    
    def test_angle_to_clock_position(self):
        """Test angle to position conversion."""
        # 0 radians = 3 o'clock
        x, y = PoseCalculator.angle_to_clock_position(0)
        self.assertAlmostEqual(x, 1.0, places=3)
        self.assertAlmostEqual(y, 0.0, places=3)
        
        # π/2 radians = 12 o'clock
        x, y = PoseCalculator.angle_to_clock_position(math.pi / 2)
        self.assertAlmostEqual(x, 0.0, places=3)
        self.assertAlmostEqual(y, 1.0, places=3)
        
        # π radians = 9 o'clock
        x, y = PoseCalculator.angle_to_clock_position(math.pi)
        self.assertAlmostEqual(x, -1.0, places=3)
        self.assertAlmostEqual(y, 0.0, places=3)


if __name__ == '__main__':
    unittest.main()