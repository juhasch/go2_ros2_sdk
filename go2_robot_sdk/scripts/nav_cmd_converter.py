"""
This script is used to convert the Nav2 command to a Twist command for the twist_mux.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class NavCmdConverter(Node):
    def __init__(self):
        super().__init__('nav_cmd_converter')
        
        # Subscribe to Nav2 commands (TwistStamped)
        self.subscription = self.create_subscription(
            TwistStamped,
            'nav_cmd_vel',
            self.nav_cmd_callback,
            10
        )
        
        # Publish clean Twist commands for twist_mux
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel_navigation',
            10
        )
        
        self.get_logger().info('Nav command converter started: nav_cmd_vel (TwistStamped) -> cmd_vel (Twist)')
    
    def nav_cmd_callback(self, msg):
        # Convert TwistStamped to Twist
        twist_msg = Twist()
        twist_msg.linear = msg.twist.linear
        twist_msg.angular = msg.twist.angular
        
        # Publish converted message
        self.publisher.publish(twist_msg)
        
        # Log the conversion for debugging
        if abs(twist_msg.linear.x) > 0.01 or abs(twist_msg.angular.z) > 0.01:
            self.get_logger().info(f'Converting Nav2 cmd: linear.x={twist_msg.linear.x:.3f}, angular.z={twist_msg.angular.z:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = NavCmdConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 