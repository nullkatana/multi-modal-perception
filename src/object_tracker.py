#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import json
import time


class KalmanFilter:
    """
    Adaptive Kalman Filter with velocity reset for handling sudden direction changes.
    State: [x, y, z, vx, vy, vz]
    """
    
    def __init__(self, initial_position, dt=0.5):
        self.state = np.array([
            initial_position[0],
            initial_position[1],
            initial_position[2],
            0.0, 0.0, 0.0
        ])
        
        self.dt = dt
        self.last_update_time = time.time()
        self.last_position = initial_position.copy()
        
        # State transition matrix
        self.F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        
        # Measurement matrix
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
        
        # CRITICAL: Very high process noise to handle sudden changes
        q = 2.0  # High value allows quick velocity adaptation
        self.Q = np.eye(6) * q
        
        # Measurement noise
        r = 0.15
        self.R = np.eye(3) * r
        
        # State covariance
        self.P = np.eye(6) * 0.5
        
    def predict(self, dt=None):
        """Predict next state"""
        if dt is not None and dt > 0:
            F_adaptive = np.array([
                [1, 0, 0, dt, 0, 0],
                [0, 1, 0, 0, dt, 0],
                [0, 0, 1, 0, 0, dt],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]
            ])
            self.state = F_adaptive @ self.state
            self.P = F_adaptive @ self.P @ F_adaptive.T + self.Q
        else:
            self.state = self.F @ self.state
            self.P = self.F @ self.P @ self.F.T + self.Q
        
        return self.state[:3]
    
    def update(self, measurement):
        """Update state with new measurement and detect direction reversals"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Calculate actual velocity from measurements
        actual_velocity = (measurement - self.last_position) / max(dt, 0.1)
        predicted_velocity = self.state[3:6]
        
        # CRITICAL FIX: Detect direction reversal
        # If velocities point in opposite directions (dot product < 0), object reversed!
        vel_dot = np.dot(actual_velocity, predicted_velocity)
        vel_magnitude = np.linalg.norm(actual_velocity)
        pred_magnitude = np.linalg.norm(predicted_velocity)
        
        direction_changed = False
        if vel_magnitude > 0.05 and pred_magnitude > 0.05:  # Both moving
            # Normalize and check angle
            cos_angle = vel_dot / (vel_magnitude * pred_magnitude)
            angle_deg = np.degrees(np.arccos(np.clip(cos_angle, -1, 1)))
            
            # Direction reversed if angle > 90 degrees
            if angle_deg > 90:
                direction_changed = True
                # RESET velocity estimate to actual measurement
                self.state[3:6] = actual_velocity * 0.5  # Use half for smoothing
                # Increase velocity uncertainty
                self.P[3:6, 3:6] = np.eye(3) * 2.0
        
        # Innovation
        y = measurement - self.H @ self.state
        innovation_magnitude = np.linalg.norm(y)
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        
        # Update covariance
        self.P = (np.eye(6) - K @ self.H) @ self.P
        
        # Update last position
        self.last_position = measurement.copy()
        
        return self.state[:3], self.state[3:6], innovation_magnitude, direction_changed
    
    def get_position(self):
        return self.state[:3]
    
    def get_velocity(self):
        return self.state[3:6]


class Track:
    """Represents a tracked object"""
    
    def __init__(self, track_id, initial_detection, dt=0.5):
        self.id = track_id
        position = np.array(initial_detection['position'])
        self.kalman = KalmanFilter(position, dt)
        self.last_detection = initial_detection
        self.missed_frames = 0
        self.hits = 1
        self.age = 0
        self.trajectory = [position.tolist()]
        self.confidence = 1.0
        self.last_update_time = time.time()
        self.last_detection_time = time.time()
        self.is_established = False
        self.direction_changes = 0  # Track how many times direction reversed
        
    def predict(self):
        """Predict next position"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        return self.kalman.predict(dt)
    
    def update(self, detection):
        """Update track with new detection"""
        position = np.array(detection['position'])
        pos, vel, innovation, direction_changed = self.kalman.update(position)
        
        if direction_changed:
            self.direction_changes += 1
        
        self.last_detection = detection
        self.last_detection_time = time.time()
        self.missed_frames = 0
        self.hits += 1
        self.trajectory.append(pos.tolist())
        self.confidence = min(1.0, self.confidence + 0.3)
        
        if self.hits >= 3:
            self.is_established = True
        
        return pos, vel, direction_changed
    
    def mark_missed(self):
        """Mark this track as having no detection this frame"""
        self.missed_frames += 1
        self.age += 1
        self.confidence = max(0.0, self.confidence - 0.08)
    
    def time_since_last_detection(self):
        return time.time() - self.last_detection_time


class ObjectTracker(Node):
    """
    Multi-object tracker with direction reversal handling.
    
    v1.4: Fixed velocity reversal issue
    - Detects when object changes direction
    - Resets velocity estimate on reversal
    - Subscribes to BOTH detections and fusion topics
    - Much larger association threshold during reversals
    """
    
    def __init__(self):
        super().__init__('object_tracker')
        
        # Subscribe to BOTH possible detection topics
        self.detection_subscription = self.create_subscription(
            String,
            'detections/objects',
            self.detection_callback,
            10
        )
        
        # FIXED: Also subscribe to fusion topic
        self.fusion_subscription = self.create_subscription(
            String,
            'fusion/detections',
            self.fusion_callback,
            10
        )
        
        self.tracking_publisher = self.create_publisher(
            String,
            'tracking/objects',
            10
        )
        
        # More forgiving parameters
        self.max_missed_frames = 20
        self.min_hits = 1
        self.base_association_threshold = 2.0  # Increased from 1.8
        self.dt = 0.5
        
        # Active tracks
        self.tracks = {}
        self.next_track_id = 1
        
        # Diagnostics
        self.total_detections = 0
        self.total_associations = 0
        self.total_direction_changes = 0
        self.last_detection_time = None
        self.detection_intervals = []
        
        self.timer = self.create_timer(0.1, self.publish_tracks)
        self.diag_timer = self.create_timer(5.0, self.print_diagnostics)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Object Tracker v1.4 (DIRECTION REVERSAL FIX) started!')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - detections/objects')
        self.get_logger().info('  - fusion/detections')
        self.get_logger().info(f'Base association threshold: {self.base_association_threshold}m')
        self.get_logger().info('=' * 60)
        
    def fusion_callback(self, msg):
        """Handle fusion detections - convert format and process"""
        try:
            data = json.loads(msg.data)
            fusion_detections = data.get('detections', [])
            
            # Convert fusion format to standard detection format
            standard_detections = []
            for det in fusion_detections:
                standard_detections.append({
                    'position': det['position'],
                    'confidence': det.get('confidence', 1.0)
                })
            
            # Process like normal detections
            self._process_detections(standard_detections)
            
        except Exception as e:
            self.get_logger().error(f'Error in fusion callback: {e}')
    
    def detection_callback(self, msg):
        """Process standard detection messages"""
        try:
            data = json.loads(msg.data)
            detections = data.get('objects', [])
            self._process_detections(detections)
        except Exception as e:
            self.get_logger().error(f'Error in detection callback: {e}')
    
    def _process_detections(self, detections):
        """Core detection processing logic"""
        current_time = time.time()
        if self.last_detection_time is not None:
            interval = current_time - self.last_detection_time
            self.detection_intervals.append(interval)
            if len(self.detection_intervals) > 20:
                self.detection_intervals.pop(0)
        self.last_detection_time = current_time
        
        self.total_detections += len(detections)
        
        if len(detections) == 0:
            for track in self.tracks.values():
                track.predict()
                track.mark_missed()
            return
        
        # Predict all tracks
        predictions = {}
        for track_id, track in self.tracks.items():
            predictions[track_id] = track.predict()
        
        # Associate with adaptive thresholds
        matched_tracks, unmatched_detections = self.associate_detections(
            detections, predictions
        )
        
        self.total_associations += len(matched_tracks)
        
        # Update matched tracks
        for track_id, detection in matched_tracks.items():
            pos, vel, direction_changed = self.tracks[track_id].update(detection)
            
            if direction_changed:
                self.total_direction_changes += 1
                self.get_logger().info(
                    f"🔄 Track #{track_id} DIRECTION REVERSED! "
                    f"New velocity: [{vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}]"
                )
            
            speed = np.linalg.norm(vel)
            track = self.tracks[track_id]
            
            if track.hits <= 5 or direction_changed:
                status = "EST" if track.is_established else "NEW"
                self.get_logger().info(
                    f"✓ Track #{track_id} [{status}] UPDATED: "
                    f"pos=[{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}], "
                    f"speed={speed:.2f}m/s, reversals={track.direction_changes}"
                )
        
        # Mark unmatched tracks as missed
        matched_ids = set(matched_tracks.keys())
        for track_id in list(self.tracks.keys()):
            if track_id not in matched_ids:
                self.tracks[track_id].mark_missed()
        
        # Create new tracks
        for detection in unmatched_detections:
            self.create_track(detection)
        
        # Delete dead tracks
        self.prune_tracks()
    
    def associate_detections(self, detections, predictions):
        """Associate detections with adaptive thresholds"""
        matched = {}
        unmatched = []
        used_track_ids = set()
        
        for detection in detections:
            det_pos = np.array(detection['position'])
            
            best_track_id = None
            best_distance = float('inf')
            
            for track_id, pred_pos in predictions.items():
                if track_id in used_track_ids:
                    continue
                
                distance = np.linalg.norm(det_pos - pred_pos)
                track = self.tracks[track_id]
                
                # ADAPTIVE THRESHOLD based on track state
                threshold = self.base_association_threshold
                
                # Established tracks get larger radius
                if track.is_established:
                    threshold *= 1.5
                
                # CRITICAL: Much larger threshold if track has recent direction changes
                if track.direction_changes > 0:
                    threshold *= 2.0  # 4x base threshold!
                
                # Increase with velocity
                vel_magnitude = np.linalg.norm(track.kalman.get_velocity())
                if vel_magnitude > 0.2:
                    threshold *= (1.0 + vel_magnitude * 0.5)
                
                if distance < threshold and distance < best_distance:
                    best_distance = distance
                    best_track_id = track_id
            
            if best_track_id is not None:
                matched[best_track_id] = detection
                used_track_ids.add(best_track_id)
            else:
                unmatched.append(detection)
                
                if len(predictions) > 0:
                    min_dist = min(np.linalg.norm(det_pos - p) for p in predictions.values())
                    if min_dist < 3.0:  # Only warn if reasonably close
                        self.get_logger().warn(
                            f"UNMATCHED detection (closest: {min_dist:.2f}m)"
                        )
        
        return matched, unmatched
    
    def create_track(self, detection):
        """Create a new track"""
        track_id = self.next_track_id
        self.tracks[track_id] = Track(track_id, detection, dt=self.dt)
        self.next_track_id += 1
        
        pos = detection['position']
        self.get_logger().info(
            f"🆕 NEW TRACK #{track_id} at [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]"
        )
    
    def prune_tracks(self):
        """Remove dead tracks"""
        to_delete = []
        
        for track_id, track in self.tracks.items():
            max_allowed = self.max_missed_frames
            if track.is_established and track.hits >= 10:
                max_allowed = int(self.max_missed_frames * 1.5)
            
            should_delete = (
                track.missed_frames > max_allowed or
                (track.missed_frames > 8 and track.confidence < 0.1)
            )
            
            if should_delete:
                to_delete.append(track_id)
        
        for track_id in to_delete:
            track = self.tracks[track_id]
            self.get_logger().info(
                f"💀 DELETED Track #{track_id} "
                f"(hits={track.hits}, reversals={track.direction_changes})"
            )
            del self.tracks[track_id]
    
    def publish_tracks(self):
        """Publish tracking state"""
        confirmed_tracks = []
        
        for track_id, track in self.tracks.items():
            if track.hits >= self.min_hits:
                pos = track.kalman.get_position()
                vel = track.kalman.get_velocity()
                
                confirmed_tracks.append({
                    'track_id': track_id,
                    'position': pos.tolist(),
                    'velocity': vel.tolist(),
                    'speed': float(np.linalg.norm(vel)),
                    'hits': track.hits,
                    'age': track.age,
                    'missed_frames': track.missed_frames,
                    'confidence': track.confidence,
                    'trajectory_length': len(track.trajectory),
                    'direction_changes': track.direction_changes
                })
        
        tracking_data = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'num_tracks': len(confirmed_tracks),
            'tracks': confirmed_tracks
        }
        
        msg = String()
        msg.data = json.dumps(tracking_data, indent=2)
        self.tracking_publisher.publish(msg)
    
    def print_diagnostics(self):
        """Print diagnostics"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('DIAGNOSTIC REPORT:')
        
        if len(self.detection_intervals) > 0:
            avg_interval = np.mean(self.detection_intervals)
            detection_rate = 1.0 / avg_interval if avg_interval > 0 else 0
            self.get_logger().info(
                f'Detection rate: {detection_rate:.2f} Hz'
            )
        
        if self.total_detections > 0:
            assoc_rate = (self.total_associations / self.total_detections) * 100
            self.get_logger().info(
                f'Association rate: {assoc_rate:.1f}% '
                f'({self.total_associations}/{self.total_detections})'
            )
        
        self.get_logger().info(f'Total direction reversals detected: {self.total_direction_changes}')
        self.get_logger().info(f'Active tracks: {len(self.tracks)}')
        
        for track_id, track in self.tracks.items():
            status = "EST" if track.is_established else "NEW"
            self.get_logger().info(
                f'  Track #{track_id} [{status}]: hits={track.hits}, '
                f'reversals={track.direction_changes}, conf={track.confidence:.2f}'
            )
        
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()