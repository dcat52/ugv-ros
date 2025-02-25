import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json

class CmdVelToWheelVelocities(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_wheel_vel_node')

        # Get wheel separation and radius from parameters (or set default values)
        self.wheel_separation = self.declare_parameter('wheel_separation', 0.3).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.05).value

        # Create a subscriber to the /cmd_vel topic
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Create a publisher for the /raw/serial_out topic
        self.serial_out_pub = self.create_publisher(
            String, 'raw/serial_write', 10)

    def cmd_vel_callback(self, msg):
        # Calculate left and right wheel velocities from Twist message
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2) / self.wheel_radius
        right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2) / self.wheel_radius

        # Create the JSON message
        data = {
            "T": 1,
            "L": left_wheel_vel/10.0,
            "R": right_wheel_vel/10.0
        }
        json_string = json.dumps(data)

        # Create and publish the ROS message
        msg = String()
        msg.data = json_string
        self.serial_out_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToWheelVelocities()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

