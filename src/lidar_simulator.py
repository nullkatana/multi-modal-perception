#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
import json

class LiDARSimulator(Node):
    """
    Simulates a 2D LiDAR sensor (like SICK TiM or Hokuyo).
    Generates 360° laser scan with configurable resolution.
    Publishes LaserScan messages for SLAM and navigation.
    """
    
    def __init__(self):
        super().__init__('lidar_simulator')
        
        # Create publisher for LiDAR data
        self.publisher = self.create_publisher(LaserScan, 'lidar/scan', 10)
        
        # Timer to publish scan data at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_scan_data)
        
        # Subscribe to shared object state
        self.state_subscription = self.create_subscription(
            String,
            'object/state',
            self.state_callback,
            10
        )
        
        # LiDAR parameters (similar to SICK TiM571)
        self.min_range = 0.05    # meters
        self.max_range = 25.0    # meters - much longer than other sensors!
        self.angle_min = -np.pi  # -180 degrees
        self.angle_max = np.pi   # +180 degrees
        self.angle_increment = np.radians(1.0)  # 1 degree resolution = 360 points
        self.scan_time = 0.1     # 10 Hz = 100ms per scan
        
        # Calculate number of range measurements
        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment)
        
        # Object state
        self.object_position = np.array([1.5, 0.0, 0.5])
        self.object_size = 0.5
        self.object_present = True
        
        self.get_logger().info('LiDAR Simulator started!')
        self.get_logger().info(f'Range: {self.min_range}m to {self.max_range}m')
        self.get_logger().info(f'Resolution: {np.degrees(self.angle_increment):.1f}° ({self.num_ranges} points per scan)')
        self.get_logger().info(f'Scan rate: {1.0/self.scan_time:.0f} Hz')
        
    def state_callback(self, msg):
        """Receive shared object state"""
        try:
            state = json.loads(msg.data)
            self.object_present = state['present']
            if self.object_present:
                self.object_position = np.array(state['position'])
                self.object_size = state.get('size', 0.5)
        except json.JSONDecodeError:
            pass
    
    def check_ray_intersection(self, angle):
        """
        Check if a laser ray at given angle intersects with the object.
        Returns distance to intersection or max_range if no hit.
        
        LiDAR scans in 2D horizontal plane (at sensor height z=0).
        """
        if not self.object_present:
            return self.max_range
        
        # Ray origin at sensor position (0, 0)
        ray_origin = np.array([0.0, 0.0])
        
        # Ray direction from angle
        ray_direction = np.array([np.cos(angle), np.sin(angle)])
        
        # Object center in 2D (x, y plane)
        object_center_2d = np.array([self.object_position[0], self.object_position[1]])
        object_radius = self.object_size / 2.0
        
        # Ray-circle intersection (2D)
        # Vector from ray origin to circle center
        oc = ray_origin - object_center_2d
        
        # Quadratic formula coefficients
        a = np.dot(ray_direction, ray_direction)
        b = 2.0 * np.dot(oc, ray_direction)
        c = np.dot(oc, oc) - object_radius * object_radius
        
        discriminant = b * b - 4 * a * c
        
        if discriminant < 0:
            return self.max_range  # No intersection
        
        # Calculate nearest intersection distance
        t = (-b - np.sqrt(discriminant)) / (2.0 * a)
        
        if t < self.min_range:
            return self.max_range  # Too close
        
        if t > self.max_range:
            return self.max_range  # Too far
        
        # Add realistic LiDAR noise (±2cm)
        noise = np.random.normal(0, 0.02)
        distance = np.clip(t + noise, self.min_range, self.max_range)
        
        return distance
    
    def generate_scan(self):
        """
        Generate complete 360° laser scan.
        Returns array of range measurements.
        """
        ranges = []
        
        # Scan from angle_min to angle_max
        angle = self.angle_min
        while angle < self.angle_max:
            distance = self.check_ray_intersection(angle)
            ranges.append(distance)
            angle += self.angle_increment
        
        return ranges
    
    def publish_scan_data(self):
        """Publish LiDAR scan"""
        # Generate scan
        ranges = self.generate_scan()
        
        # Create LaserScan message
        msg = LaserScan()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_frame'
        
        # Scan parameters
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = self.scan_time / len(ranges)
        msg.scan_time = self.scan_time
        msg.range_min = self.min_range
        msg.range_max = self.max_range
        
        # Range data
        msg.ranges = ranges
        
        # Intensity (not simulated, set to empty)
        msg.intensities = []
        
        # Publish
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LiDARSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()