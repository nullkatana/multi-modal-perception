#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class TFBroadcaster(Node):
    """
    Broadcasts TF transforms to define all sensor positions in 3D space.
    This allows RViz2 to visualize all sensor frames correctly.
    """
    
    def __init__(self):
        super().__init__('tf_broadcaster')
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to broadcast transforms at 50 Hz
        self.timer = self.create_timer(0.02, self.broadcast_transforms)
        
        self.get_logger().info('TF Broadcaster started!')
        self.get_logger().info('Broadcasting transforms for all sensors:')
        self.get_logger().info('  - world -> ir_sensor_frame')
        self.get_logger().info('  - world -> nir_sensor_frame')
        self.get_logger().info('  - world -> tof_sensor_frame')
        self.get_logger().info('  - world -> lidar_frame')
        
    def create_transform(self, frame_id, x, y, z, yaw=0.0):
        """
        Helper function to create a TransformStamped message.
        
        Args:
            frame_id: Child frame name
            x, y, z: Position in meters
            yaw: Rotation around Z-axis in radians (optional)
        """
        t = TransformStamped()
        
        # Header
        # Add small time buffer to ensure TF is always ahead
        current_time = self.get_clock().now()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = frame_id
        
        # Translation
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        # Rotation (yaw only, around Z-axis)
        # Convert yaw to quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        
        return t
        
    def broadcast_transforms(self):
        """Broadcast transforms for all sensor frames"""
        
        transforms = []
        
        # IR Sensor - at origin, pointing forward (+X)
        transforms.append(self.create_transform('ir_sensor_frame', 0.0, 0.0, 0.0))
        
        # NIR Sensor - slightly offset to the right, pointing forward
        # Positioned 5cm to the right of IR sensor
        transforms.append(self.create_transform('nir_sensor_frame', 0.0, 0.05, 0.0))
        
        # ToF Array - above and between IR/NIR, pointing forward
        # Positioned 10cm above origin
        transforms.append(self.create_transform('tof_sensor_frame', 0.0, 0.0, 0.1))
        
        # LiDAR - at origin (same as IR), pointing forward
        # LiDAR scans in horizontal plane at sensor height
        transforms.append(self.create_transform('lidar_frame', 0.0, 0.0, 0.0))
        
        # Broadcast all transforms
        for transform in transforms:
            self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()