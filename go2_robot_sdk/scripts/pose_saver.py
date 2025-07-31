#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import json
import os
import signal
import sys

class PoseSaver(Node):
    def __init__(self):
        super().__init__('pose_saver')
        
        # File to save pose
        self.pose_file = os.path.expanduser('~/last_robot_pose.json')
        
        # TF listener for getting current pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher for setting initial pose
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        # Timer to periodically save pose
        self.save_timer = self.create_timer(10.0, self.save_current_pose)
        
        # Setup signal handler to save pose on shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Restore pose on startup
        self.restore_timer = self.create_timer(3.0, self.restore_pose_once)
        
        self.get_logger().info(f"Pose saver started. Saving to: {self.pose_file}")
    
    def save_current_pose(self):
        """Save current robot pose to file"""
        try:
            # Get transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            
            pose_data = {
                'x': transform.transform.translation.x,
                'y': transform.transform.translation.y,
                'z': transform.transform.translation.z,
                'qx': transform.transform.rotation.x,
                'qy': transform.transform.rotation.y,
                'qz': transform.transform.rotation.z,
                'qw': transform.transform.rotation.w,
                'timestamp': self.get_clock().now().to_msg().sec
            }
            
            with open(self.pose_file, 'w') as f:
                json.dump(pose_data, f, indent=2)
                
        except TransformException as e:
            # Don't spam logs - robot might not be localized yet
            pass
    
    def restore_pose_once(self):
        """Restore saved pose on startup (run once)"""
        self.restore_timer.cancel()  # Only run once
        
        if not os.path.exists(self.pose_file):
            self.get_logger().info("No saved pose found - will need manual initial pose")
            return
            
        try:
            with open(self.pose_file, 'r') as f:
                pose_data = json.load(f)
            
            # Create initial pose message
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.header.frame_id = 'map'
            
            # Set position
            initial_pose.pose.pose.position.x = pose_data['x']
            initial_pose.pose.pose.position.y = pose_data['y']
            initial_pose.pose.pose.position.z = pose_data['z']
            
            # Set orientation
            initial_pose.pose.pose.orientation.x = pose_data['qx']
            initial_pose.pose.pose.orientation.y = pose_data['qy']
            initial_pose.pose.pose.orientation.z = pose_data['qz']
            initial_pose.pose.pose.orientation.w = pose_data['qw']
            
            # Set covariance (small values = confident)
            initial_pose.pose.covariance[0] = 0.25   # x
            initial_pose.pose.covariance[7] = 0.25   # y
            initial_pose.pose.covariance[35] = 0.06854  # yaw
            
            # Publish initial pose
            self.initial_pose_pub.publish(initial_pose)
            
            age = self.get_clock().now().to_msg().sec - pose_data['timestamp']
            self.get_logger().info(f"✅ Restored robot pose from {age}s ago: ({pose_data['x']:.2f}, {pose_data['y']:.2f})")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to restore pose: {e}")
    
    def signal_handler(self, sig, frame):
        """Save pose on shutdown"""
        self.get_logger().info("Shutting down - saving final pose...")
        self.save_current_pose()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = PoseSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_current_pose()  # Save on exit
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 