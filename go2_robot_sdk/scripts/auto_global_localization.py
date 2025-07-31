#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time

class AutoGlobalLocalization(Node):
    def __init__(self):
        super().__init__('auto_global_localization')
        
        # Wait for AMCL to be ready
        self.get_logger().info("Waiting for AMCL global localization service...")
        self.global_loc_client = self.create_client(Empty, '/global_localization')
        
        # Wait for service to be available
        while not self.global_loc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Global localization service not available, waiting...')
        
        # Trigger global localization after a delay
        self.timer = self.create_timer(5.0, self.trigger_global_localization)
        
    def trigger_global_localization(self):
        """Automatically trigger global localization"""
        self.get_logger().info("Triggering global localization - spreading particles across map...")
        
        request = Empty.Request()
        future = self.global_loc_client.call_async(request)
        future.add_done_callback(self.global_loc_callback)
        
        # Only run once
        self.timer.cancel()
    
    def global_loc_callback(self, future):
        """Handle global localization response"""
        try:
            response = future.result()
            self.get_logger().info("✅ Global localization triggered! Robot will auto-localize as it moves.")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to trigger global localization: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AutoGlobalLocalization()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 