"""
ROS2 node for GUI-based pose issuing.
"""

import tkinter as tk
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty

from .clock_widget import ClockWidget


class GuiPoseIssuer(Node):
    """ROS2 node that provides a GUI for commanding robot poses."""
    
    def __init__(self):
        super().__init__('gui_pose_issuer')
        
        # Create publishers
        self.pose_publisher = self.create_publisher(
            Pose, '/user/pose_command', 10
        )
        self.cancel_publisher = self.create_publisher(
            Empty, '/user/pose_cancel', 10
        )
        
        # Initialize GUI in main thread
        self.root = None
        self.clock_widget = None
        self.gui_thread = None
        
        self.get_logger().info('GUI Pose Issuer node started')
        
        # Start GUI
        self._setup_gui()
    
    def _setup_gui(self):
        """Set up the Tkinter GUI."""
        self.root = tk.Tk()
        self.root.title('Robot Pose Controller')
        self.root.resizable(False, False)
        
        # Create main frame
        main_frame = tk.Frame(self.root)
        main_frame.pack(padx=10, pady=10)
        
        # Add title
        title_label = tk.Label(
            main_frame, 
            text='Robot Pose Controller',
            font=('Arial', 16, 'bold')
        )
        title_label.pack(pady=(0, 10))
        
        # Create clock widget
        self.clock_widget = ClockWidget(main_frame, size=400)
        self.clock_widget.set_click_callback(self._on_pose_clicked)
        self.clock_widget.set_cancel_callback(self._on_cancel_clicked)
        
        # Set up window close handler
        self.root.protocol('WM_DELETE_WINDOW', self._on_window_close)
        
        # Give focus to clock widget for keyboard events
        self.root.after(100, self.clock_widget.focus)
    
    def _on_pose_clicked(self, pose: Pose):
        """Handle pose selection from clock widget."""
        try:
            # Publish the pose command
            self.pose_publisher.publish(pose)
            
            self.get_logger().info(
                f'Published pose command: x={pose.position.x:.3f}, '
                f'y={pose.position.y:.3f}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to publish pose: {e}')
    
    def _on_cancel_clicked(self):
        """Handle cancel command (spacebar press)."""
        try:
            # Publish cancel message
            cancel_msg = Empty()
            self.cancel_publisher.publish(cancel_msg)
            
            self.get_logger().info('Published cancel command')
        except Exception as e:
            self.get_logger().error(f'Failed to publish cancel: {e}')
    
    def _on_window_close(self):
        """Handle window close event."""
        self.get_logger().info('GUI window closing')
        self.root.quit()
    
    def run_gui(self):
        """Run the GUI main loop."""
        if self.root:
            self.root.mainloop()
    
    def shutdown_gui(self):
        """Shutdown the GUI."""
        if self.root:
            self.root.quit()


def main(args=None):
    """Main entry point for the GUI pose issuer node."""
    rclpy.init(args=args)
    
    try:
        # Create the node
        node = GuiPoseIssuer()
        
        # Run ROS2 spinning in a separate thread
        ros_thread = threading.Thread(
            target=lambda: rclpy.spin(node),
            daemon=True
        )
        ros_thread.start()
        
        # Run GUI in main thread (required for Tkinter)
        node.run_gui()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in GUI pose issuer: {e}')
    finally:
        # Cleanup
        if 'node' in locals():
            node.shutdown_gui()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()