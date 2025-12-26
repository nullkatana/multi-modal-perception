#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
import numpy as np
import json

class NIRSensorSimulator(Node):
    """
    Simulates a Near-Infrared (NIR) distance sensor.
    NIR sensors have better penetration through fog/dust and slightly different
    noise characteristics compared to standard IR sensors.
    """
    
    def __init__(self):
        super().__init__('nir_sensor_simulator')
        
        # Create publisher for NIR sensor data
        self.publisher = self.create_publisher(Range, 'nir_sensor/range', 10)
        
        # Timer to publish sensor data at 12 Hz (slightly faster than IR)
        self.timer = self.create_timer(1.0/12.0, self.publish_sensor_data)
        
        # Subscribe to shared object state
        self.state_subscription = self.create_subscription(
            String,
            'object/state',
            self.state_callback,
            10
        )
        
        # Simulation parameters (NIR specific)
        self.min_range = 0.15  # meters (slightly longer minimum range)
        self.max_range = 3.5   # meters (slightly better max range)
        self.fov = 0.08        # field of view in radians (~4.6 degrees, narrower than IR)
        
        # Simulated object position (distance from sensor)
        self.object_distance = 1.5  # meters
        self.object_present = True
        
        self.get_logger().info('NIR Sensor Simulator started!')
        self.get_logger().info(f'Range: {self.min_range}m to {self.max_range}m')
        self.get_logger().info('Characteristics: Better fog/dust penetration, lower noise')
        
    def state_callback(self, msg):
        """Receive shared object state"""
        try:
            state = json.loads(msg.data)
            self.object_present = state['present']
            if self.object_present:
                # Calculate distance from sensor (at origin) to object
                position = np.array(state['position'])
                self.object_distance = position[0]  # X-distance (sensor points along X)
        except json.JSONDecodeError:
            pass
        
    def publish_sensor_data(self):
        """Publish simulated NIR sensor reading"""
        msg = Range()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'nir_sensor_frame'
        
        # Sensor characteristics
        msg.radiation_type = Range.INFRARED  # NIR is still infrared spectrum
        msg.field_of_view = self.fov
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        
        # Simulate distance reading with NIR-specific noise
        if self.object_present:
            # NIR has lower noise (σ = 1.5cm vs IR's 2cm)
            noise = np.random.normal(0, 0.015)  # 1.5cm standard deviation
            measured_distance = self.object_distance + noise
            
            # Clamp to valid range
            measured_distance = max(self.min_range, min(self.max_range, measured_distance))
            msg.range = measured_distance
        else:
            # No object detected - return max range
            msg.range = self.max_range
        
        # Publish the message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NIRSensorSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()