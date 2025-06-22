"""
Tkinter-based clock widget for pose selection.
"""

import tkinter as tk
import math
from typing import Callable, Optional
from .pose_calculator import PoseCalculator


class ClockWidget:
    """A simple clock interface for selecting robot poses."""
    
    def __init__(self, parent: tk.Widget, size: int = 400):
        """
        Initialize the clock widget.
        
        Args:
            parent: Parent tkinter widget
            size: Size of the clock widget in pixels
        """
        self.size = size
        self.radius = size // 2 - 20  # Leave margin for hour markers
        self.center_x = size // 2
        self.center_y = size // 2
        
        # Callbacks
        self.on_click_callback: Optional[Callable] = None
        self.on_cancel_callback: Optional[Callable] = None
        
        # Create canvas
        self.canvas = tk.Canvas(parent, width=size, height=size, bg='white')
        self.canvas.pack(padx=10, pady=10)
        
        # Bind events
        self.canvas.bind('<Button-1>', self._on_canvas_click)
        self.canvas.focus_set()  # Allow canvas to receive keyboard events
        self.canvas.bind('<KeyPress-space>', self._on_space_press)
        
        # Draw the clock
        self._draw_clock()
    
    def set_click_callback(self, callback: Callable):
        """Set callback function for clock clicks."""
        self.on_click_callback = callback
    
    def set_cancel_callback(self, callback: Callable):
        """Set callback function for cancel (spacebar) events."""
        self.on_cancel_callback = callback
    
    def _draw_clock(self):
        """Draw the clock face with hour markers."""
        # Clear canvas
        self.canvas.delete('all')
        
        # Draw main circle
        x1 = self.center_x - self.radius
        y1 = self.center_y - self.radius
        x2 = self.center_x + self.radius
        y2 = self.center_y + self.radius
        self.canvas.create_oval(x1, y1, x2, y2, outline='black', width=2)
        
        # Draw hour markers
        for hour in range(12):
            angle = hour * math.pi / 6 - math.pi / 2  # Start at 12 o'clock
            
            # Calculate marker positions
            inner_radius = self.radius - 15
            outer_radius = self.radius - 5
            
            x1 = self.center_x + inner_radius * math.cos(angle)
            y1 = self.center_y + inner_radius * math.sin(angle)
            x2 = self.center_x + outer_radius * math.cos(angle)
            y2 = self.center_y + outer_radius * math.sin(angle)
            
            # Draw marker line
            self.canvas.create_line(x1, y1, x2, y2, fill='black', width=2)
            
            # Add hour numbers
            text_radius = self.radius - 25
            text_x = self.center_x + text_radius * math.cos(angle)
            text_y = self.center_y + text_radius * math.sin(angle)
            
            hour_text = str(12 if hour == 0 else hour)
            self.canvas.create_text(text_x, text_y, text=hour_text, 
                                  font=('Arial', 12, 'bold'))
        
        # Draw center dot
        center_size = 4
        self.canvas.create_oval(
            self.center_x - center_size, self.center_y - center_size,
            self.center_x + center_size, self.center_y + center_size,
            fill='black'
        )
        
        # Add instructions
        instruction_text = "Click on clock to command position\nPress SPACE to cancel"
        self.canvas.create_text(
            self.center_x, self.size - 30,
            text=instruction_text,
            font=('Arial', 10),
            justify=tk.CENTER
        )
    
    def _on_canvas_click(self, event):
        """Handle mouse clicks on the canvas."""
        click_x = event.x
        click_y = event.y
        
        # Convert to pose and get clipped coordinates
        if self.on_click_callback:
            pose, (clipped_x, clipped_y) = PoseCalculator.clock_position_to_pose(
                click_x, click_y, self.center_x, self.center_y, self.radius
            )
            self.on_click_callback(pose)
            
            # Visual feedback - draw marker at clipped position (on unit circle)
            self._draw_click_indicator(clipped_x, clipped_y)
    
    def _on_space_press(self, event):
        """Handle spacebar press for canceling commands."""
        if self.on_cancel_callback:
            self.on_cancel_callback()
            
        # Clear any click indicators
        self._clear_click_indicators()
    
    def _draw_click_indicator(self, x: float, y: float):
        """Draw a visual indicator at the clicked position."""
        # Clear previous indicators
        self._clear_click_indicators()
        
        # Draw new indicator
        indicator_size = 6
        self.canvas.create_oval(
            x - indicator_size, y - indicator_size,
            x + indicator_size, y + indicator_size,
            fill='red', outline='darkred', width=2,
            tags='click_indicator'
        )
    
    def _clear_click_indicators(self):
        """Clear all click indicator graphics."""
        self.canvas.delete('click_indicator')
    
    def focus(self):
        """Give focus to the canvas for keyboard events."""
        self.canvas.focus_set()