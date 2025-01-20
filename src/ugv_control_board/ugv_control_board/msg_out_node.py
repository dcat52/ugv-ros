# msg_out_node.py
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import String message type

class RawMsgOutNode(Node):
    def __init__(self):
        super().__init__('msg_out_node')
        self.get_logger().info("Starting Message Out Node")

        # Create a publisher to send raw messages
        self.msg_command_pub = self.create_publisher(
            String, 'raw/serial_out', 40)

        self.raw_msg_in_sub = self.create_subscription(
            String, 'raw/msg_out', self.raw_msg_in_callback, 40)

    def raw_msg_in_callback(self, msg):

      data = {'T': 1004, 'mac': "FF:FF:FF:FF:FF:FF", 'status':1, 'megs':""}
      dest = ""
      s = ""
      try:
         msg_dict = json.loads(msg_str)
         dest = msg_dict['mac']
         data['megs'] = msg_dict['megs']
      except Exception as e:
         self.get_logger().warn(f'{e}, Sending data as broadcast')
         dest = "FF:FF:FF:FF:FF:FF"
         data['megs'] = msg.data
      finally:
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