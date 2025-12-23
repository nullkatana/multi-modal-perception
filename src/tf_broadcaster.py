#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class TFBroadcaster(Node):
    """
    Broadcasts TF transforms to define sensor positions in 3D space.
    This allows RViz2 to visualize sensor frames correctly.
    """
    
    def __init__(self):
        super().__init__('tf_broadcaster')
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to broadcast transforms at 10 Hz
        self.timer = self.create_timer(0.1, self.broadcast_transforms)
        
        self.get_logger().info('TF Broadcaster started!')
        self.get_logger().info('Broadcasting transforms: world -> ir_sensor_frame')
        
    def broadcast_transforms(self):
        """Broadcast the transform from world to ir_sensor_frame"""
        
        # Create transform message
        t = TransformStamped()
        
        # Header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'  # Parent frame
        t.child_frame_id = 'ir_sensor_frame'  # Child frame
        
        # Translation (position in meters)
        # IR sensor is positioned at the origin, looking forward along X-axis
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Rotation (orientation as quaternion)
        # Sensor points along positive X-axis (forward)
        # No rotation needed from world frame
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0  # No rotation (identity quaternion)
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)


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