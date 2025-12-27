#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import json


class SharedObjectState(Node):
    """
    Publishes shared object state for synchronized sensor simulation.
    All sensors subscribe to this to ensure they're sensing the same object.
    
    v1.5: Added timed movement patterns:
    - 0-20s: Random movement
    - 20-40s: Horizontal oscillation (left-right)
    - 40-60s: Forward-backward oscillation
    - Then repeats
    """
    
    def __init__(self):
        super().__init__('shared_object_state')
        
        # Publisher for object state
        self.state_publisher = self.create_publisher(String, 'object/state', 10)
        
        # Timer to update object state at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_object_state)
        
        # Object shapes available
        self.available_shapes = ['sphere', 'box', 'cylinder', 'cone']
        self.current_shape = 'sphere'
        
        # Object parameters
        self.object_position = np.array([1.5, 0.0, 0.5])  # [x, y, z] in meters
        self.object_velocity = np.array([0.0, 0.0, 0.0])
        self.object_size = 0.5
        self.object_present = True
        
        # Movement parameters
        self.min_distance = 0.5
        self.max_distance = 2.5
        self.max_speed = 0.3
        self.change_direction_probability = 0.1
        self.disappear_probability = 0.05
        self.change_shape_probability = 0.25
        
        # Timed pattern parameters
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.phase_duration = 20.0  # Each phase lasts 20 seconds
        
        # Oscillation parameters
        self.oscillation_step = 0
        self.oscillation_direction = 1
        self.steps_per_direction = 5
        
        # Center positions for oscillations
        self.center_x = 1.5  # Forward-backward center
        self.center_y = 0.0  # Left-right center
        
        self.get_logger().info('Shared Object State publisher started!')
        self.get_logger().info(f'Initial object position: {self.object_position}')
        self.get_logger().info(f'Movement pattern: 0-20s random, 20-40s horizontal, 40-60s forward-back')
        
    def get_current_phase(self):
        """Determine which movement phase we're in (0, 1, or 2)"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time
        cycle_time = elapsed % (3 * self.phase_duration)  # 60 second cycle
        
        if cycle_time < self.phase_duration:
            return 0  # Random movement
        elif cycle_time < 2 * self.phase_duration:
            return 1  # Horizontal oscillation
        else:
            return 2  # Forward-backward oscillation
    
    def update_random_movement(self):
        """Phase 0: Random movement (original behavior)"""
        # Check for disappear/reappear
        if np.random.random() < self.disappear_probability:
            self.object_present = not self.object_present
            if self.object_present:
                self.get_logger().info('Object appeared')
            else:
                self.get_logger().info('Object disappeared')
        
        # Randomly change shape
        if self.object_present and np.random.random() < self.change_shape_probability:
            old_shape = self.current_shape
            self.current_shape = np.random.choice(self.available_shapes)
            if old_shape != self.current_shape:
                self.get_logger().info(f'Object shape changed: {old_shape} → {self.current_shape}')
        
        # Update movement if present
        if self.object_present:
            # Randomly change direction
            if np.random.random() < self.change_direction_probability:
                self.object_velocity = np.array([
                    np.random.uniform(-self.max_speed, self.max_speed),
                    np.random.uniform(-self.max_speed, self.max_speed),
                    0.0
                ])
                
                speed = np.linalg.norm(self.object_velocity)
                if speed > 0.01:
                    self.get_logger().info(f'Object velocity changed: speed={speed:.2f}m/s')
            
            # Update position
            dt = 1.0
            self.object_position += self.object_velocity * dt
            
            # Clamp position
            self.object_position[0] = np.clip(self.object_position[0], self.min_distance, self.max_distance)
            self.object_position[1] = np.clip(self.object_position[1], -1.5, 1.5)
            self.object_position[2] = np.clip(self.object_position[2], 0.3, 0.7)
            
            # Bounce off boundaries
            if self.object_position[0] <= self.min_distance or self.object_position[0] >= self.max_distance:
                self.object_velocity[0] *= -1
            if self.object_position[1] <= -1.5 or self.object_position[1] >= 1.5:
                self.object_velocity[1] *= -1
    
    def update_horizontal_oscillation(self):
        """Phase 1: Move left-right continuously (5 steps each direction)"""
        self.object_present = True  # Ensure object is present
        self.object_velocity = np.array([0.0, 0.0, 0.0])  # Stop random velocity
        
        # Move to center X if needed
        if abs(self.object_position[0] - self.center_x) > 0.1:
            self.object_position[0] = self.center_x
        
        # Oscillate horizontally
        step_size = 0.3  # Size of each step in meters
        self.object_position[1] += self.oscillation_direction * step_size
        
        self.oscillation_step += 1
        
        # Reverse direction after 5 steps
        if self.oscillation_step >= self.steps_per_direction:
            self.oscillation_direction *= -1
            self.oscillation_step = 0
            direction_name = "right" if self.oscillation_direction > 0 else "left"
            self.get_logger().info(f'Horizontal oscillation: moving {direction_name}')
        
        # Keep within bounds
        self.object_position[1] = np.clip(self.object_position[1], -1.5, 1.5)
        self.object_position[2] = 0.5  # Keep at center height
    
    def update_forward_backward_oscillation(self):
        """Phase 2: Move forward-backward continuously (5 steps each direction)"""
        self.object_present = True  # Ensure object is present
        self.object_velocity = np.array([0.0, 0.0, 0.0])  # Stop random velocity
        
        # Move to center Y if needed
        if abs(self.object_position[1] - self.center_y) > 0.1:
            self.object_position[1] = self.center_y
        
        # Oscillate forward-backward
        step_size = 0.3  # Size of each step in meters
        self.object_position[0] += self.oscillation_direction * step_size
        
        self.oscillation_step += 1
        
        # Reverse direction after 5 steps
        if self.oscillation_step >= self.steps_per_direction:
            self.oscillation_direction *= -1
            self.oscillation_step = 0
            direction_name = "forward" if self.oscillation_direction > 0 else "backward"
            self.get_logger().info(f'Forward-backward oscillation: moving {direction_name}')
        
        # Keep within bounds
        self.object_position[0] = np.clip(self.object_position[0], self.min_distance, self.max_distance)
        self.object_position[2] = 0.5  # Keep at center height
        
    def publish_object_state(self):
        """Publish current object state based on current phase"""
        
        phase = self.get_current_phase()
        
        # Log phase changes
        if not hasattr(self, 'last_phase'):
            self.last_phase = -1
        
        if phase != self.last_phase:
            phase_names = ["Random Movement", "Horizontal Oscillation", "Forward-Backward Oscillation"]
            self.get_logger().info(f'=== Phase {phase}: {phase_names[phase]} ===')
            self.last_phase = phase
            self.oscillation_step = 0  # Reset oscillation counter
            self.oscillation_direction = 1  # Reset direction
        
        # Execute movement based on phase
        if phase == 0:
            self.update_random_movement()
        elif phase == 1:
            self.update_horizontal_oscillation()
        elif phase == 2:
            self.update_forward_backward_oscillation()
        
        # Create state message
        state = {
            'present': self.object_present,
            'position': self.object_position.tolist(),
            'size': self.object_size,
            'shape': self.current_shape,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        
        # Publish
        msg = String()
        msg.data = json.dumps(state)
        self.state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SharedObjectState()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()