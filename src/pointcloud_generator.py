#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
import numpy as np
import struct
import json


class PointCloudGenerator(Node):
    """
    Generates simulated 3D point cloud data with objects.
    Publishes PointCloud2 messages representing a scene with detectable objects.
    
    v1.4: Added support for multiple object shapes (box, sphere, cylinder, cone)
    """
    
    def __init__(self):
        super().__init__('pointcloud_generator')
        
        # Create publisher for point cloud data
        self.publisher = self.create_publisher(PointCloud2, 'pointcloud/scene', 10)
        
        # Timer to publish point cloud at 5 Hz
        self.timer = self.create_timer(0.2, self.publish_pointcloud)
        
        # Scene parameters
        self.scene_width = 5.0  # meters
        self.scene_depth = 5.0  # meters
        self.scene_height = 3.0  # meters
        
        # Object properties
        self.object_position = np.array([1.5, 0.0, 0.5])  # [x, y, z]
        self.object_size = 0.5  # meters
        self.object_shape = 'sphere'  # NEW in v1.4

        # Subscribe to shared object state
        self.state_subscription = self.create_subscription(
            String,
            'object/state',
            self.state_callback,
            10
        )
        self.object_present = True
        
        self.get_logger().info('Point Cloud Generator started!')
        self.get_logger().info(f'Scene size: {self.scene_width}x{self.scene_depth}x{self.scene_height}m')
        self.get_logger().info('Supports shapes: sphere, box, cylinder, cone')
        
    def generate_ground_plane(self, num_points=500):
        """Generate points for the ground plane"""
        x = np.random.uniform(-self.scene_width/2, self.scene_width/2, num_points)
        y = np.random.uniform(-self.scene_depth/2, self.scene_depth/2, num_points)
        z = np.zeros(num_points)  # Ground at z=0
        
        return np.column_stack([x, y, z])
    
    def generate_sphere(self, center, radius, num_points=200):
        """Generate points for a sphere using spherical coordinates"""
        points = []
        for _ in range(num_points):
            # Random spherical coordinates
            theta = np.random.uniform(0, 2 * np.pi)  # Azimuthal angle
            phi = np.random.uniform(0, np.pi)  # Polar angle
            r = radius * np.cbrt(np.random.uniform(0, 1))  # Uniform distribution in volume
            
            # Convert to Cartesian
            x = r * np.sin(phi) * np.cos(theta)
            y = r * np.sin(phi) * np.sin(theta)
            z = r * np.cos(phi)
            
            points.append([x, y, z])
        
        points = np.array(points)
        points += center  # Translate to position
        return points
    
    def generate_box(self, center, size, num_points=200):
        """Generate points for a rectangular box"""
        # Random points within a cube
        points = np.random.uniform(-size/2, size/2, (num_points, 3))
        points += center  # Translate to position
        return points
    
    def generate_cylinder(self, center, radius, height, num_points=200):
        """Generate points for a cylinder (vertical orientation)"""
        points = []
        for _ in range(num_points):
            # Random cylindrical coordinates
            r = radius * np.sqrt(np.random.uniform(0, 1))  # Uniform in circle
            theta = np.random.uniform(0, 2 * np.pi)
            z = np.random.uniform(-height/2, height/2)
            
            # Convert to Cartesian
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            
            points.append([x, y, z])
        
        points = np.array(points)
        points += center  # Translate to position
        return points
    
    def generate_cone(self, center, base_radius, height, num_points=200):
        """Generate points for a cone (pointing upward)"""
        points = []
        for _ in range(num_points):
            # Random height within cone
            z = np.random.uniform(0, height)
            
            # Radius decreases linearly with height
            max_r = base_radius * (1 - z / height)
            r = max_r * np.sqrt(np.random.uniform(0, 1))
            theta = np.random.uniform(0, 2 * np.pi)
            
            # Convert to Cartesian
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            
            points.append([x, y, z - height/2])  # Center the cone vertically
        
        points = np.array(points)
        points += center  # Translate to position
        return points
    
    def generate_object(self, center, size, shape='sphere', num_points=200):
        """
        Generate points for an object based on shape.
        
        Args:
            center: [x, y, z] position
            size: Base size parameter (interpreted differently per shape)
            shape: 'sphere', 'box', 'cylinder', or 'cone'
            num_points: Number of points to generate
        """
        if shape == 'sphere':
            return self.generate_sphere(center, size/2, num_points)
        
        elif shape == 'box':
            return self.generate_box(center, size, num_points)
        
        elif shape == 'cylinder':
            radius = size / 2
            height = size * 1.2  # Slightly taller than wide
            return self.generate_cylinder(center, radius, height, num_points)
        
        elif shape == 'cone':
            base_radius = size / 2
            height = size * 1.5  # Pointy!
            return self.generate_cone(center, base_radius, height, num_points)
        
        else:
            # Default to sphere if unknown shape
            self.get_logger().warn(f'Unknown shape "{shape}", defaulting to sphere')
            return self.generate_sphere(center, size/2, num_points)
    
    def generate_noise_points(self, num_points=50):
        """Generate random noise points (simulating sensor noise)"""
        x = np.random.uniform(-self.scene_width/2, self.scene_width/2, num_points)
        y = np.random.uniform(-self.scene_depth/2, self.scene_depth/2, num_points)
        z = np.random.uniform(0, self.scene_height, num_points)
        
        return np.column_stack([x, y, z])
    
    def create_pointcloud2_msg(self, points):
        """Convert numpy array of points to PointCloud2 message"""
        msg = PointCloud2()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        # Point cloud dimensions
        msg.height = 1  # Unorganized point cloud
        msg.width = len(points)
        
        # Point fields (x, y, z)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        # Pack point data
        buffer = []
        for point in points:
            buffer.append(struct.pack('fff', point[0], point[1], point[2]))
        
        msg.data = b''.join(buffer)
        
        return msg
    
    def publish_pointcloud(self):
        """Generate and publish a complete point cloud scene"""
        # Generate different components of the scene
        ground = self.generate_ground_plane(num_points=500)
        noise = self.generate_noise_points(num_points=50)
        
        # Only generate object if present (v1.4: now uses shape parameter)
        if self.object_present:
            obj = self.generate_object(
                self.object_position, 
                self.object_size, 
                self.object_shape,  # NEW: Use current shape
                num_points=200
            )
            all_points = np.vstack([ground, obj, noise])
        else:
            all_points = np.vstack([ground, noise])
        
        # Create and publish message
        msg = self.create_pointcloud2_msg(all_points)
        self.publisher.publish(msg)

    def state_callback(self, msg):
        """Receive shared object state (v1.4: now reads shape field)"""
        try:
            state = json.loads(msg.data)
            self.object_present = state['present']
            if self.object_present:
                self.object_position = np.array(state['position'])
                # NEW in v1.4: Read shape from state
                self.object_shape = state.get('shape', 'sphere')  # Default to sphere if not present
        except json.JSONDecodeError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()