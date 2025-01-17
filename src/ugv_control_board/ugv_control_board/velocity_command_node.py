# velocity_command_node.py
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String  # Import String message type

class VelocityCommandNode(Node):
    def __init__(self):
        super().__init__('velocity_command_node')
        self.get_logger().info("Starting Velocity Command Node")

        # Create a publisher to send velocity commands
        self.velocity_command_pub = self.create_publisher(
            String, 'raw/serial_out', 10)

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):

        # Create JSON message
        data = {'T': 13, 'X': msg.linear.x, 'Z': msg.angular.z}

        # Create a message for the velocity_command topic
        msg = String()
        msg.data = json.dumps(data)

        # Publish the message
        self.velocity_command_pub.publish(msg)

    def send_command_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Command sent successfully")
            else:
                self.get_logger().error(
                    f"Failed to send command: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    velocity_command_node = VelocityCommandNode()
    rclpy.spin(velocity_command_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()