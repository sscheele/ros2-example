"""
Tests for the GUI pose issuer ROS2 node.
"""

import unittest
from unittest.mock import Mock, patch
import rclpy
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty


class TestGuiPoseIssuer(unittest.TestCase):
    """Test cases for GuiPoseIssuer node."""
    
    @classmethod
    def setUpClass(cls):
        """Set up ROS2 for testing."""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2 after testing."""
        rclpy.shutdown()
    
    def setUp(self):
        """Set up test fixtures."""
        # Mock tkinter and related GUI components during node instantiation
        with patch.dict('sys.modules', {'tkinter': Mock()}):
            with patch('gui_pose_issuer.gui_pose_issuer_node.ClockWidget'):
                with patch('tkinter.Tk'):
                    # Import the class after mocking to avoid inheritance issues
                    from gui_pose_issuer.gui_pose_issuer_node import GuiPoseIssuer
                    self.node = GuiPoseIssuer()
    
    def tearDown(self):
        """Clean up after each test."""
        self.node.destroy_node()
    
    def test_node_initialization(self):
        """Test that node initializes correctly."""
        self.assertEqual(self.node.get_name(), 'gui_pose_issuer')
        self.assertIsNotNone(self.node.pose_publisher)
        self.assertIsNotNone(self.node.cancel_publisher)
    
    def test_pose_publishing(self):
        """Test pose message publishing."""
        # Create a test pose
        test_pose = Pose()
        test_pose.position.x = 0.5
        test_pose.position.y = 0.866  # Approximately 60 degrees
        test_pose.position.z = 0.0
        test_pose.orientation.w = 1.0
        
        # Mock the publisher
        self.node.pose_publisher.publish = Mock()
        
        # Call the pose clicked handler
        self.node._on_pose_clicked(test_pose)
        
        # Verify publish was called with correct pose
        self.node.pose_publisher.publish.assert_called_once_with(test_pose)
    
    def test_cancel_publishing(self):
        """Test cancel message publishing."""
        # Mock the publisher
        self.node.cancel_publisher.publish = Mock()
        
        # Call the cancel clicked handler
        self.node._on_cancel_clicked()
        
        # Verify publish was called with Empty message
        self.node.cancel_publisher.publish.assert_called_once()
        args = self.node.cancel_publisher.publish.call_args[0]
        self.assertIsInstance(args[0], Empty)
    
    def test_pose_publisher_topic(self):
        """Test that pose publisher uses correct topic."""
        # Check topic name - get_publisher_names_and_types_by_node returns list of tuples (name, types)
        topic_info = self.node.get_publisher_names_and_types_by_node(
            self.node.get_name(), self.node.get_namespace()
        )
        topic_names = [topic[0] for topic in topic_info]
        self.assertIn('/user/pose_command', topic_names)
    
    def test_cancel_publisher_topic(self):
        """Test that cancel publisher uses correct topic."""
        # Check topic name - get_publisher_names_and_types_by_node returns list of tuples (name, types)
        topic_info = self.node.get_publisher_names_and_types_by_node(
            self.node.get_name(), self.node.get_namespace()
        )
        topic_names = [topic[0] for topic in topic_info]
        self.assertIn('/user/pose_cancel', topic_names)
    
    def test_error_handling_pose(self):
        """Test error handling in pose publishing."""
        # Mock publisher to raise exception
        self.node.pose_publisher.publish = Mock(side_effect=Exception("Test error"))
        
        # Mock the logger
        self.node.get_logger = Mock()
        mock_logger = Mock()
        self.node.get_logger.return_value = mock_logger
        
        # Create test pose
        test_pose = Pose()
        
        # Call handler - should not raise exception
        self.node._on_pose_clicked(test_pose)
        
        # Verify error was logged
        mock_logger.error.assert_called()
    
    def test_error_handling_cancel(self):
        """Test error handling in cancel publishing."""
        # Mock publisher to raise exception
        self.node.cancel_publisher.publish = Mock(side_effect=Exception("Test error"))
        
        # Mock the logger
        self.node.get_logger = Mock()
        mock_logger = Mock()
        self.node.get_logger.return_value = mock_logger
        
        # Call handler - should not raise exception
        self.node._on_cancel_clicked()
        
        # Verify error was logged
        mock_logger.error.assert_called()


if __name__ == '__main__':
    unittest.main()