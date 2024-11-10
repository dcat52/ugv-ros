#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial  
import json  

# Open the sereial port   
ser = serial.Serial('/dev/serial0', 115200, timeout=1)  

class CmdVelSubscribe(Node):
    def __init__(self, name):
        super().__init__(name)
        # Create the subscriber  
        self.command_subscribe_ = self.create_subscription(Twist, "cmd_vel", self.command_callback, 10)
        
    def command_callback(self, msg):
        print("msg.linear.x:  ", msg.linear.x)  
        print("msg.angular.z:  ", msg.angular.z)  
        angular_velocity = msg.angular.z
        # Prevents rotating in place at speeds too low to rotate
        if msg.linear.x == 0:
            if 0<angular_velocity<0.2:
                angular_velocity = 0.2
            elif -0.2<angular_velocity<0:
                angular_velocity = -0.2

        # Create JSON messages 
        data = {'T': '13', 'X': msg.linear.x, 'Z': angular_velocity}  

        json_data = json.dumps(data).encode('utf-8')
        
        # Send the JSON messge to the serial port  
        ser.write(json_data + b'\n') 

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    node = CmdVelSubscribe("robot_control")  # Create a node
    rclpy.spin(node) # Keep the nodes running and test whether receive the command to exit (Ctrl+C)
    rclpy.shutdown() # turn off rclpy
    # Turn off the serial port 
    ser.close()
