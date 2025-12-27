#!/usr/bin/env python3
"""
ML Object Classifier Node

Real-time shape classification using trained neural network.
Subscribes to detections and classifies objects using point cloud data.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import numpy as np
import struct
import json
import torch
import torch.nn as nn
import torch.nn.functional as F
from pathlib import Path


class PointNetClassifier(nn.Module):
    """
    Simplified PointNet architecture for point cloud classification.
    Input: (batch_size, num_points, 3)
    Output: (batch_size, num_classes)
    """
    
    def __init__(self, num_classes=4):
        super(PointNetClassifier, self).__init__()
        
        # Shared MLP layers
        self.conv1 = nn.Conv1d(3, 64, 1)
        self.conv2 = nn.Conv1d(64, 128, 1)
        self.conv3 = nn.Conv1d(128, 256, 1)
        
        # Classification head
        self.fc1 = nn.Linear(256, 128)
        self.fc2 = nn.Linear(128, 64)
        self.fc3 = nn.Linear(64, num_classes)
        
        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(128)
        self.bn3 = nn.BatchNorm1d(256)
        self.bn4 = nn.BatchNorm1d(128)
        self.bn5 = nn.BatchNorm1d(64)
        
        self.dropout = nn.Dropout(p=0.3)
        
    def forward(self, x):
        # x shape: (batch_size, num_points, 3)
        # Transpose to (batch_size, 3, num_points) for Conv1d
        x = x.transpose(2, 1)
        
        # Shared MLP
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        
        # Global max pooling
        x = torch.max(x, 2)[0]  # (batch_size, 256)
        
        # Classification MLP
        x = F.relu(self.bn4(self.fc1(x)))
        x = self.dropout(x)
        x = F.relu(self.bn5(self.fc2(x)))
        x = self.dropout(x)
        x = self.fc3(x)
        
        return x


class MLObjectClassifier(Node):
    """
    ROS 2 node for real-time object classification.
    """
    
    def __init__(self):
        super().__init__('ml_object_classifier')
        
        # Class names
        self.class_names = ['sphere', 'box', 'cylinder', 'cone']
        self.num_classes = len(self.class_names)
        
        # Device
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Using device: {self.device}')
        
        # Load model
        self.model = PointNetClassifier(num_classes=self.num_classes).to(self.device)
        model_path = Path.home() / 'multi_modal_perception' / 'models' / 'object_classifier.pth'
        
        if model_path.exists():
            self.model.load_state_dict(torch.load(model_path, map_location=self.device))
            self.model.eval()
            self.get_logger().info(f'Loaded model from: {model_path}')
        else:
            self.get_logger().warn(f'Model not found at {model_path}')
            self.get_logger().warn('Please train the model first using scripts/train_model.py')
            self.get_logger().warn('Continuing with untrained model (random predictions)')
        
        # Current point cloud and object state
        self.latest_pointcloud = None
        self.object_position = None
        self.object_present = False
        
        # Subscribers
        self.pc_subscription = self.create_subscription(
            PointCloud2, 'pointcloud/scene', self.pointcloud_callback, 10)
        
        self.state_subscription = self.create_subscription(
            String, 'object/state', self.state_callback, 10)
        
        # Publisher
        self.classification_publisher = self.create_publisher(
            String, 'classified/objects', 10)
        
        # Classification timer (2 Hz)
        self.timer = self.create_timer(0.5, self.classify_object)
        
        self.get_logger().info('ML Object Classifier started!')
        self.get_logger().info(f'Classes: {", ".join(self.class_names)}')
        
    def parse_pointcloud(self, pc_msg):
        """Convert PointCloud2 message to numpy array"""
        points = []
        point_step = pc_msg.point_step
        
        for i in range(0, len(pc_msg.data), point_step):
            x, y, z = struct.unpack('fff', pc_msg.data[i:i+12])
            points.append([x, y, z])
        
        return np.array(points) if points else np.array([])
    
    def extract_object_points(self, points, object_position, radius=1.0):
        """Extract points belonging to the object"""
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
        """Normalize point cloud to unit sphere"""
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
    
    def prepare_input(self, points):
        """Prepare point cloud for neural network"""
        if len(points) == 0:
            return None
        
        # Normalize
        normalized = self.normalize_pointcloud(points)
        
        # Resample to 200 points
        if len(normalized) > 200:
            indices = np.random.choice(len(normalized), 200, replace=False)
            sample = normalized[indices]
        else:
            padding = np.zeros((200 - len(normalized), 3))
            sample = np.vstack([normalized, padding])
        
        # Convert to tensor
        tensor = torch.from_numpy(sample).float().unsqueeze(0)  # (1, 200, 3)
        return tensor.to(self.device)
    
    def state_callback(self, msg):
        """Update object state"""
        try:
            state = json.loads(msg.data)
            self.object_present = state['present']
            if self.object_present:
                self.object_position = np.array(state['position'])
        except json.JSONDecodeError:
            pass
    
    def pointcloud_callback(self, msg):
        """Store latest point cloud"""
        self.latest_pointcloud = self.parse_pointcloud(msg)
    
    def classify_object(self):
        """Classify the current object"""
        if not self.object_present or self.latest_pointcloud is None:
            return
        
        # Extract object points
        object_points = self.extract_object_points(
            self.latest_pointcloud, 
            self.object_position
        )
        
        if len(object_points) < 50:
            return
        
        # Prepare input
        input_tensor = self.prepare_input(object_points)
        if input_tensor is None:
            return
        
        # Run inference
        with torch.no_grad():
            output = self.model(input_tensor)
            probabilities = F.softmax(output, dim=1)
            confidence, predicted = torch.max(probabilities, 1)
            
            predicted_class = self.class_names[predicted.item()]
            confidence_score = confidence.item()
            
            # Get all class probabilities
            all_probs = probabilities[0].cpu().numpy()
        
        # Create classification result
        result = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'position': self.object_position.tolist(),
            'predicted_class': predicted_class,
            'confidence': float(confidence_score),
            'probabilities': {
                self.class_names[i]: float(all_probs[i]) 
                for i in range(self.num_classes)
            },
            'num_points': len(object_points)
        }
        
        # Publish
        msg = String()
        msg.data = json.dumps(result, indent=2)
        self.classification_publisher.publish(msg)
        
        # Log
        self.get_logger().info(
            f'Classification: {predicted_class.upper()} '
            f'(confidence: {confidence_score:.2%}, points: {len(object_points)})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MLObjectClassifier()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()