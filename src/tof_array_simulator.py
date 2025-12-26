#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String, Header
import numpy as np
import struct
import json

class ToFArraySimulator(Node):
    """
    Simulates a Time-of-Flight (ToF) array sensor (like VL53L5CX).
    Generates an 8x8 grid of distance measurements (64 points total).
    Publishes as a structured point cloud representing a depth image.
    """
    
    def __init__(self):
        super().__init__('tof_array_simulator')
        
        # Create publisher for ToF array data
        self.publisher = self.create_publisher(PointCloud2, 'tof_array/pointcloud', 10)
        
        # Timer to publish sensor data at 20 Hz (ToF arrays are fast!)
        self.timer = self.create_timer(0.05, self.publish_sensor_data)
        
        # Subscribe to shared object state
        self.state_subscription = self.create_subscription(
            String,
            'object/state',
            self.state_callback,
            10
        )
        
        # Simulation parameters
        self.min_range = 0.05  # meters - ToF can measure very close
        self.max_range = 4.0   # meters
        self.fov_horizontal = 45.0  # degrees - wide field of view
        self.fov_vertical = 45.0    # degrees
        
        # Grid parameters (8x8 like VL53L5CX)
        self.grid_width = 8
        self.grid_height = 8
        self.total_pixels = self.grid_width * self.grid_height
        
        # Object state
        self.object_position = np.array([1.5, 0.0, 0.5])
        self.object_size = 0.5
        self.object_present = True
        
        self.get_logger().info('ToF Array Simulator started!')
        self.get_logger().info(f'Grid: {self.grid_width}x{self.grid_height} = {self.total_pixels} pixels')
        self.get_logger().info(f'Range: {self.min_range}m to {self.max_range}m')
        self.get_logger().info(f'FOV: {self.fov_horizontal}° × {self.fov_vertical}°')
        
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
    
    def ray_intersects_object(self, ray_direction):
        """
        Check if a ray intersects with the object (sphere approximation).
        Returns distance to intersection or None if no hit.
        """
        if not self.object_present:
            return None
        
        # Ray origin is at sensor position (0, 0, 0)
        ray_origin = np.array([0.0, 0.0, 0.0])
        
        # Sphere center and radius
        sphere_center = self.object_position
        sphere_radius = self.object_size / 2.0
        
        # Ray-sphere intersection (simplified)
        oc = ray_origin - sphere_center
        a = np.dot(ray_direction, ray_direction)
        b = 2.0 * np.dot(oc, ray_direction)
        c = np.dot(oc, oc) - sphere_radius * sphere_radius
        discriminant = b * b - 4 * a * c
        
        if discriminant < 0:
            return None  # No intersection
        
        # Calculate intersection distance
        t = (-b - np.sqrt(discriminant)) / (2.0 * a)
        
        if t < self.min_range or t > self.max_range:
            return None
        
        return t
    
    def generate_depth_grid(self):
        """
        Generate 8x8 grid of distance measurements.
        Returns array of (x, y, z) points representing the depth image.
        """
        points = []
        
        # Convert FOV to radians
        fov_h_rad = np.radians(self.fov_horizontal)
        fov_v_rad = np.radians(self.fov_vertical)
        
        # Calculate angular step between pixels
        h_step = fov_h_rad / (self.grid_width - 1)
        v_step = fov_v_rad / (self.grid_height - 1)
        
        # Start angles (center the FOV)
        h_start = -fov_h_rad / 2.0
        v_start = -fov_v_rad / 2.0
        
        for row in range(self.grid_height):
            for col in range(self.grid_width):
                # Calculate ray angles for this pixel
                h_angle = h_start + col * h_step
                v_angle = v_start + row * v_step
                
                # Convert angles to ray direction (sensor points along +X)
                # X = forward, Y = horizontal, Z = vertical
                ray_direction = np.array([
                    np.cos(v_angle) * np.cos(h_angle),  # X
                    np.cos(v_angle) * np.sin(h_angle),  # Y
                    np.sin(v_angle)                      # Z
                ])
                ray_direction = ray_direction / np.linalg.norm(ray_direction)
                
                # Check for object intersection
                distance = self.ray_intersects_object(ray_direction)
                
                if distance is None:
                    # No object hit - return max range
                    distance = self.max_range
                else:
                    # Add realistic ToF noise (±1cm for close range)
                    noise = np.random.normal(0, 0.01)
                    distance = np.clip(distance + noise, self.min_range, self.max_range)
                
                # Calculate 3D point position
                point_3d = ray_direction * distance
                points.append(point_3d)
        
        return np.array(points)
    
    def create_pointcloud2_msg(self, points):
        """Convert numpy array to PointCloud2 message"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'tof_sensor_frame'
        
        # Define point cloud fields (x, y, z)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Pack points into binary format
        cloud_data = []
        for point in points:
            cloud_data.append(struct.pack('fff', point[0], point[1], point[2]))
        
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = header
        msg.height = self.grid_height
        msg.width = self.grid_width
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * self.grid_width
        msg.is_dense = True
        msg.data = b''.join(cloud_data)
        
        return msg
    
    def publish_sensor_data(self):
        """Publish ToF array depth grid"""
        # Generate depth measurements
        points = self.generate_depth_grid()
        
        # Create and publish message
        msg = self.create_pointcloud2_msg(points)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ToFArraySimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()