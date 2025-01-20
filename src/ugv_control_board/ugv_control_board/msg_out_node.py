# msg_out_node.py
import copy

import json

import json_messages

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RawMsgOutNode(Node):
    def __init__(self):
        super().__init__('msg_out_node')
        self.get_logger().info("Starting Message Out Node")

        # Create a publisher to send raw messages
        self.msg_command_pub = self.create_publisher(
            String, 'raw/serial_write', 40)

        self.raw_msg_in_sub = self.create_subscription(
            String, 'raw/msg_out', self.raw_msg_out_callback, 40)

    def raw_msg_out_callback(self, msg):
        self.get_logger().info("Sending Msg:")
        self.get_logger().info(msg.data)

        data = copy.copy(json_messages.MSG_TEMPLATE)
        dest = ""
        s = ""
        try:
            msg_dict = json.loads(msg.data)
            dest = msg_dict['mac']
            s = msg_dict['megs']
        except Exception as e:
            self.get_logger().warn(f'{e}, Sending data as broadcast')
            dest = "FF:FF:FF:FF:FF:FF"
            s = msg.data
        finally:
            data['mac'] = dest
            data['megs'] = s
            send_msg = String()
            data_str = json.dumps(data)
            self.get_logger().info(data_str)
            send_msg.data = data_str
            self.msg_command_pub.publish(send_msg)

def main(args=None):
    rclpy.init(args=args)
    raw_msg_out_node = RawMsgOutNode()
    rclpy.spin(raw_msg_out_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
