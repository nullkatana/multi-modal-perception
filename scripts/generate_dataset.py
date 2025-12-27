#!/usr/bin/env python3
"""
Synthetic Dataset Generator for ML Object Classification

Generates labeled point cloud samples for training a shape classifier.
Records point clouds from the simulation and saves them with their labels.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import numpy as np
import struct
import json
import os
from pathlib import Path


class DatasetGenerator(Node):
    """
    Subscribes to point cloud and object state topics.
    Extracts object point clouds and saves them with shape labels.
    """
    
    def __init__(self, output_dir, samples_per_class=250):
        super().__init__('dataset_generator')
        
        # Output directory
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Target samples per shape
        self.samples_per_class = samples_per_class
        self.shapes = ['sphere', 'box', 'cylinder', 'cone']
        
        # Create subdirectories for each shape
        for shape in self.shapes:
            (self.output_dir / shape).mkdir(exist_ok=True)
        
        # Sample counters
        self.sample_counts = {shape: 0 for shape in self.shapes}
        self.total_samples = 0
        
        # Current object state
        self.current_shape = None
        self.object_present = False
        self.object_position = None
        
        # Subscribers
        self.pc_subscription = self.create_subscription(
            PointCloud2, 'pointcloud/scene', self.pointcloud_callback, 10)
        
        self.state_subscription = self.create_subscription(
            String, 'object/state', self.state_callback, 10)
        
        self.get_logger().info('Dataset Generator started!')
        self.get_logger().info(f'Output directory: {self.output_dir.absolute()}')
        self.get_logger().info(f'Target: {samples_per_class} samples per class')
        self.get_logger().info(f'Total target: {samples_per_class * len(self.shapes)} samples')
        self.get_logger().info('Collecting data...')
        
    def parse_pointcloud(self, pc_msg):
        """Convert PointCloud2 message to numpy array"""
        points = []
        point_step = pc_msg.point_step
        
        for i in range(0, len(pc_msg.data), point_step):
            x, y, z = struct.unpack('fff', pc_msg.data[i:i+12])
            points.append([x, y, z])
        
        return np.array(points) if points else np.array([])
    
    def extract_object_points(self, points, object_position, radius=1.0):
        """
        Extract points belonging to the object.
        Filters points within radius of object position and above ground.
        """
        if len(points) == 0 or object_position is None:
            return np.array([])
        
        # Filter points above ground
        elevated = points[points[:, 2] > 0.1]
        
        if len(elevated) == 0:
            return np.array([])
        
        # Calculate distances to object center
        distances = np.linalg.norm(elevated - object_position, axis=1)
        
        # Keep points within radius
        object_points = elevated[distances < radius]
        
        return object_points
    
    def normalize_pointcloud(self, points):
        """
        Normalize point cloud to unit sphere centered at origin.
        Makes the classifier scale and position invariant.
        """
        if len(points) == 0:
            return points
        
        # Center at origin
        centroid = np.mean(points, axis=0)
        centered = points - centroid
        
        # Scale to unit sphere
        max_dist = np.max(np.linalg.norm(centered, axis=1))
        if max_dist > 0:
            normalized = centered / max_dist
        else:
            normalized = centered
        
        return normalized
    
    def state_callback(self, msg):
        """Update current object state"""
        try:
            state = json.loads(msg.data)
            self.object_present = state['present']
            self.current_shape = state.get('shape', None)
            if self.object_present:
                self.object_position = np.array(state['position'])
        except json.JSONDecodeError:
            pass
    
    def pointcloud_callback(self, msg):
        """Process point cloud and save samples"""
        # Check if we need more samples for this shape
        if not self.object_present or self.current_shape is None:
            return
        
        if self.sample_counts[self.current_shape] >= self.samples_per_class:
            # Check if all classes are complete
            if all(count >= self.samples_per_class for count in self.sample_counts.values()):
                self.get_logger().info('Dataset generation complete!')
                self.print_summary()
                rclpy.shutdown()
            return
        
        # Parse point cloud
        points = self.parse_pointcloud(msg)
        
        if len(points) == 0:
            return
        
        # Extract object points
        object_points = self.extract_object_points(points, self.object_position)
        
        # Need at least 50 points for a valid sample
        if len(object_points) < 50:
            return
        
        # Normalize point cloud
        normalized = self.normalize_pointcloud(object_points)
        
        # Resample to fixed size (200 points)
        if len(normalized) > 200:
            # Randomly sample 200 points
            indices = np.random.choice(len(normalized), 200, replace=False)
            sample = normalized[indices]
        else:
            # Pad with zeros if not enough points
            padding = np.zeros((200 - len(normalized), 3))
            sample = np.vstack([normalized, padding])
        
        # Save sample
        sample_id = self.sample_counts[self.current_shape]
        filename = self.output_dir / self.current_shape / f'sample_{sample_id:04d}.npy'
        np.save(filename, sample)
        
        # Update counters
        self.sample_counts[self.current_shape] += 1
        self.total_samples += 1
        
        # Log progress every 10 samples
        if self.total_samples % 10 == 0:
            self.get_logger().info(
                f'Progress: {self.total_samples}/{self.samples_per_class * len(self.shapes)} | '
                f'sphere:{self.sample_counts["sphere"]} '
                f'box:{self.sample_counts["box"]} '
                f'cylinder:{self.sample_counts["cylinder"]} '
                f'cone:{self.sample_counts["cone"]}'
            )
    
    def print_summary(self):
        """Print final summary"""
        self.get_logger().info('='*50)
        self.get_logger().info('Dataset Generation Summary')
        self.get_logger().info('='*50)
        for shape in self.shapes:
            count = self.sample_counts[shape]
            path = self.output_dir / shape
            self.get_logger().info(f'{shape:10s}: {count:4d} samples in {path}')
        self.get_logger().info('-'*50)
        self.get_logger().info(f'Total: {self.total_samples} samples')
        self.get_logger().info(f'Saved to: {self.output_dir.absolute()}')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    
    # Output directory
    output_dir = os.path.expanduser('~/multi_modal_perception/data/training_dataset')
    
    # Number of samples per shape (250 each = 1000 total)
    samples_per_class = 250
    
    node = DatasetGenerator(output_dir, samples_per_class)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
        node.print_summary()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()