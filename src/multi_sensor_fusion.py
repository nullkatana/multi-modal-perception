#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, PointCloud2, LaserScan
from std_msgs.msg import String
import numpy as np
import struct
import json
from sklearn.cluster import DBSCAN


class MultiSensorFusion(Node):
    """
    Advanced multi-sensor fusion detector.
    Combines data from 4 sensor types plus 3D scene representation:
    - IR sensor (single-point infrared)
    - NIR sensor (single-point near-infrared)
    - ToF array (8x8 depth grid)
    - LiDAR (360° laser scan)
    Plus:
    - Point Cloud (full 3D scene representation)
    
    Uses weighted fusion based on sensor confidence and reliability.
    """
    
    def __init__(self):
        super().__init__('multi_sensor_fusion')
        
        # Subscribers for all sensors
        self.ir_subscription = self.create_subscription(
            Range, 'ir_sensor/range', self.ir_callback, 10)
        
        self.nir_subscription = self.create_subscription(
            Range, 'nir_sensor/range', self.nir_callback, 10)
        
        self.tof_subscription = self.create_subscription(
            PointCloud2, 'tof_array/pointcloud', self.tof_callback, 10)
        
        self.lidar_subscription = self.create_subscription(
            LaserScan, 'lidar/scan', self.lidar_callback, 10)
        
        self.pc_subscription = self.create_subscription(
            PointCloud2, 'pointcloud/scene', self.pointcloud_callback, 10)
        
        # Publisher for fusion results
        self.fusion_publisher = self.create_publisher(
            String, 'fusion/detections', 10)
        
        # Store latest sensor data
        self.latest_ir = None
        self.latest_nir = None
        self.latest_tof = None
        self.latest_lidar = None
        self.latest_pointcloud = None
        
        # Sensor confidence weights (tunable)
        # Note: Point cloud gets its own weight as scene representation
        self.weights = {
            'ir': 0.15,        # Low weight - single point, noisy
            'nir': 0.20,       # Slightly better - less noise
            'tof': 0.25,       # Good - 64 points, structured
            'lidar': 0.20,     # Good - 360° coverage
            'pointcloud': 0.20 # Good - full 3D scene, but slower
        }
        
        # DBSCAN parameters for point clouds
        self.dbscan_eps = 0.35
        self.dbscan_min_samples = 8
        self.detection_threshold = 0.02
        
        # Fusion timer
        self.fusion_timer = self.create_timer(0.5, self.perform_fusion)
        
        self.get_logger().info('Multi-Sensor Fusion Detector started!')
        self.get_logger().info('Subscribing to 4 sensors + Point Cloud 3D Scene:')
        self.get_logger().info('  Sensors:')
        self.get_logger().info('    - IR Sensor (single-point)')
        self.get_logger().info('    - NIR Sensor (single-point)')
        self.get_logger().info('    - ToF Array (8x8 grid)')
        self.get_logger().info('    - LiDAR (360° scan)')
        self.get_logger().info('  Scene Data:')
        self.get_logger().info('    - Point Cloud (3D scene representation)')
        self.get_logger().info(f'Fusion weights: {self.weights}')
        
    def ir_callback(self, msg):
        self.latest_ir = msg.range
        
    def nir_callback(self, msg):
        self.latest_nir = msg.range
        
    def tof_callback(self, msg):
        self.latest_tof = self.parse_pointcloud(msg)
        
    def lidar_callback(self, msg):
        self.latest_lidar = msg.ranges
        
    def pointcloud_callback(self, msg):
        self.latest_pointcloud = self.parse_pointcloud(msg)
    
    def parse_pointcloud(self, pc_msg):
        """Convert PointCloud2 message to numpy array"""
        points = []
        point_step = pc_msg.point_step
        
        for i in range(0, len(pc_msg.data), point_step):
            x, y, z = struct.unpack('fff', pc_msg.data[i:i+12])
            points.append([x, y, z])
        
        return np.array(points) if points else np.array([])
    
    def detect_objects_dbscan(self, points):
        """Apply DBSCAN clustering to point cloud"""
        if len(points) == 0:
            return []
        
        # Filter elevated points
        elevated = points[points[:, 2] > 0.1]
        
        if len(elevated) < self.dbscan_min_samples:
            return []
        
        try:
            clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples)
            labels = clustering.fit_predict(elevated)
        except Exception as e:
            return []
        
        unique_labels = set(labels)
        if -1 in unique_labels:
            unique_labels.remove(-1)
        
        detections = []
        for label in unique_labels:
            cluster = elevated[labels == label]
            centroid = np.mean(cluster, axis=0)
            size = np.max(cluster, axis=0) - np.min(cluster, axis=0)
            volume = size[0] * size[1] * size[2]
            
            if volume > self.detection_threshold:
                detections.append({
                    'position': centroid.tolist(),
                    'size': size.tolist(),
                    'volume': float(volume),
                    'points': len(cluster),
                    'distance': float(centroid[0])
                })
        
        return detections
    
    def get_lidar_detections(self):
        """Extract object detections from LiDAR scan"""
        if self.latest_lidar is None:
            return []
        
        detections = []
        ranges = np.array(self.latest_lidar)
        
        # Find clusters of nearby ranges (objects)
        valid = ranges < 24.0  # Ignore max range readings
        
        if not np.any(valid):
            return []
        
        # Simple clustering: consecutive valid ranges
        in_cluster = False
        cluster_start = 0
        
        for i, is_valid in enumerate(valid):
            if is_valid and not in_cluster:
                cluster_start = i
                in_cluster = True
            elif not is_valid and in_cluster:
                # End of cluster
                cluster_ranges = ranges[cluster_start:i]
                if len(cluster_ranges) > 5:  # At least 5 consecutive readings
                    avg_distance = np.mean(cluster_ranges)
                    angle = -np.pi + (cluster_start + len(cluster_ranges)/2) * np.radians(1.0)
                    
                    # Convert polar to cartesian
                    x = avg_distance * np.cos(angle)
                    y = avg_distance * np.sin(angle)
                    
                    detections.append({
                        'position': [x, y, 0.0],
                        'distance': float(avg_distance),
                        'angle': float(np.degrees(angle)),
                        'points': len(cluster_ranges)
                    })
                
                in_cluster = False
        
        return detections
    
    def fuse_detections(self, ir_dist, nir_dist, tof_dets, lidar_dets, pc_dets):
        """
        Weighted fusion of 4 sensor measurements plus point cloud scene data.
        Returns consolidated detection results with confidence scores.
        """
        fused = []
        
        # Collect all distance estimates with their sources
        distance_estimates = []
        
        # Sensor 1: IR
        if ir_dist is not None and ir_dist < 3.0:
            distance_estimates.append(('ir', ir_dist, self.weights['ir']))
        
        # Sensor 2: NIR
        if nir_dist is not None and nir_dist < 3.5:
            distance_estimates.append(('nir', nir_dist, self.weights['nir']))
        
        # Sensor 3: ToF array detections
        for det in tof_dets:
            distance_estimates.append(('tof', det['distance'], self.weights['tof']))
        
        # Sensor 4: LiDAR detections
        for det in lidar_dets:
            distance_estimates.append(('lidar', det['distance'], self.weights['lidar']))
        
        # Scene representation: Point cloud detections (most detailed)
        for det in pc_dets:
            distance_estimates.append(('pointcloud', det['distance'], self.weights['pointcloud']))
        
        if not distance_estimates:
            return []
        
        # Weighted average of all distance estimates
        total_weight = sum(w for _, _, w in distance_estimates)
        weighted_distance = sum(d * w for _, d, w in distance_estimates) / total_weight
        
        # Calculate confidence based on number of contributing data sources
        num_sources = len(set(source for source, _, _ in distance_estimates))
        confidence = min(num_sources / 5.0, 1.0)  # Max confidence when all 5 sources agree
        
        # Find best position estimate (prefer point cloud, then ToF, then LiDAR)
        position = None
        if pc_dets:
            position = pc_dets[0]['position']
        elif tof_dets:
            position = tof_dets[0]['position']
        elif lidar_dets:
            position = lidar_dets[0]['position']
        else:
            # Estimate from distance (assume straight ahead)
            position = [weighted_distance, 0.0, 0.5]
        
        fused.append({
            'position': position,
            'distance': float(weighted_distance),
            'confidence': float(confidence),
            'num_contributing_sources': num_sources,
            'contributing_sources': [s for s, _, _ in distance_estimates]
        })
        
        return fused
    
    def perform_fusion(self):
        """Main fusion loop"""
        # Extract detections from each sensor
        tof_dets = self.detect_objects_dbscan(self.latest_tof) if self.latest_tof is not None else []
        lidar_dets = self.get_lidar_detections()
        pc_dets = self.detect_objects_dbscan(self.latest_pointcloud) if self.latest_pointcloud is not None else []
        
        # Fuse all sensor data and point cloud scene
        fused_detections = self.fuse_detections(
            self.latest_ir,
            self.latest_nir,
            tof_dets,
            lidar_dets,
            pc_dets
        )
        
        # Create fusion report
        report = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'data_sources_active': {
                'sensors': {
                    'ir': self.latest_ir is not None,
                    'nir': self.latest_nir is not None,
                    'tof': self.latest_tof is not None,
                    'lidar': self.latest_lidar is not None
                },
                'scene': {
                    'pointcloud': self.latest_pointcloud is not None
                }
            },
            'num_detections': len(fused_detections),
            'detections': fused_detections
        }
        
        # Publish
        msg = String()
        msg.data = json.dumps(report, indent=2)
        self.fusion_publisher.publish(msg)
        
        # Log detections
        if fused_detections:
            for i, det in enumerate(fused_detections):
                self.get_logger().info(
                    f"Fused Object {i+1}: Distance={det['distance']:.2f}m, "
                    f"Confidence={det['confidence']:.2f}, "
                    f"Sources={det['num_contributing_sources']}/5 "
                    f"({', '.join(det['contributing_sources'])})"
                )


def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorFusion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()