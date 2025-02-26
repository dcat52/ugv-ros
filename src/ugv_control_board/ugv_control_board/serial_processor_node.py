#!/usr/bin/env python3
# serial_processor_node.py
import traceback

import math

import numpy as np

import json

import serial

import json_messages

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

from std_msgs.msg import String, Float32, Int32MultiArray
from sensor_msgs.msg import Imu, BatteryState
from nav_msgs.msg import Odometry

from tf_transformations import quaternion_from_euler

# Determine the GPIO Serial Device Name (adapt as needed)
def is_raspberry_pi5():
    with open('/proc/cpuinfo', 'r') as file:
        for line in file:
            if 'Model' in line:
                if 'Raspberry Pi 5' in line:
                    return True
                else:
                    return False

if is_raspberry_pi5():
    serial_port = '/dev/ttyAMA0' 
else:
    serial_port = '/dev/serial0'

class SerialProcessorNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Starting Serial Processor Node")

        # Open the serial port
        try:
            self.ser = serial.Serial(serial_port, 115200, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.get_logger().error(traceback.format_exc())
            return  # Exit if serial port cannot be opened

        self.local_commands_sub = self.create_subscription(String, 'raw/local_cmds', self.raw_command_callback, 10)

        # Create a subscriber to receive commands
        self.serial_write_sub = self.create_subscription(
            String, 'raw/serial_write', self.serial_write_callback, 100)

        # Create a timer to periodically read serial data
        self.serial_read_timer = self.create_timer(0.001, self.read_serial_data)

        # Create a timer to periodically request IMU data
        self.imu_request_timer = self.create_timer(0.1, lambda: self.serial_write_json(json_messages.IMU_DATA_REQUEST1)) 
        self.imu_request_timer = self.create_timer(0.1, lambda: self.serial_write_json(json_messages.IMU_DATA_REQUEST2)) 

        # Create publishers for different data types
        self.raw_serial_msg_pub = self.create_publisher(String, 'raw/serial_in', 100)
        # self.raw_left_wheel_vel_pub = self.create_publisher(Float32, 'raw/left_wheel_velocity', 40)
        # self.raw_right_wheel_vel_pub = self.create_publisher(Float32, 'raw/right_wheel_velocity', 40)
        self.raw_roll_pitch_pub = self.create_publisher(Int32MultiArray, 'raw/roll_pitch', 10)
        # self.raw_voltage_pub = self.create_publisher(Float32, 'raw/battery_voltage', 40)
        self.raw_imu_pub = self.create_publisher(Imu, 'raw/imu_data', 10)
        self.raw_esp_now_msg_pub = self.create_publisher(String, 'raw/msg_in', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.raw_theta_pub = self.create_publisher(Float32, 'raw/theta', 10)

        self.serial_write_json(json_messages.DISABLE_STREAM_CHASSIS_INFO)
        self.serial_write_json(json_messages.ADD_ANY_BROADCAST_PEER)
        self.serial_write_json(json_messages.DISABLE_UART_WRITE_ECHO)

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0

        self.imu_calibration_data = {
            'x_scale': 1.004, 'y_scale': 1.006, 'z_scale': 0.977, 
            'x_offset': 0.0, 'y_offset': 0.0, 'z_offset': 0.0, 
        }

    def read_serial_data(self):
        try:
            # Check if there is data available on the serial port
            if self.ser.in_waiting > 0:
                line = self.ser.readline()  # Read raw bytes
                # print(f"Raw data: {line}")  # Print raw bytes
                line = line.decode('utf-8')  # Decode

                if line == '':
                    return

                ros_string = String()
                ros_string.data = line
                self.raw_serial_msg_pub.publish(ros_string)

                data = json.loads(line)  # Parse the decoded line
                self.process_serial_data(data)

        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            self.get_logger().warn(f"Str: {line}")
            self.get_logger().warn(f"Failed to parse serial data: {e}")
        except Exception as e:  # Catch any other exceptions
            self.get_logger().warn(line)
            self.get_logger().warn(f"Failed to read or decode serial data: {e}") 
            self.get_logger().warn(traceback.format_exc())

    def serial_write_callback(self, msg):
        # Get the command string from the message
        command = msg.data

        # Send the command to the serial port
        try:
            self.ser.write(command.encode('utf-8') + b'\n')
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")
            self.get_logger().error(traceback.format_exc())

    def raw_command_callback(self, msg):
        data = json.loads(msg.data)

        if data['T'] == 127:
            self.imu_calibration_step(data['step'])

        elif data['T'] == 128:
            print(data['msg'])

    def process_serial_data(self, data):

        if data['T'] == 1001:
            try:
                # # Publish left wheel velocity
                # left_wheel_msg = Float32()
                # left_wheel_msg.data = float(data['L'])
                # self.raw_left_wheel_vel_pub.publish(left_wheel_msg)

                # # Publish right wheel velocity
                # right_wheel_msg = Float32()
                # right_wheel_msg.data = float(data['R'])
                # self.raw_right_wheel_vel_pub.publish(right_wheel_msg)

                # Publish IMU data
                intarr_msg = Int32MultiArray()
                arr = []
                arr.append(int(data['r']))
                arr.append(int(data['p']))
                intarr_msg.data = arr
                self.raw_roll_pitch_pub.publish(intarr_msg)

                # Get the current time
                current_time = self.get_clock().now()

                # Calculate time elapsed since the last measurement
                if hasattr(self, 'last_measurement_time'):
                    dt = (current_time - self.last_measurement_time).nanoseconds / 1e9  # Convert to seconds
                else:
                    dt = 0.1  # Use a default value for the first measurement

                self.last_measurement_time = current_time  # Update last measurement time

                # Get left and right track velocities
                left_track_vel = float(data['L'])
                right_track_vel = float(data['R'])

                wheel_distance = 0.18
                # Calculate linear and angular velocity
                linear_velocity = (left_track_vel + right_track_vel) / 2.0
                angular_velocity = (right_track_vel - left_track_vel) / wheel_distance

                # # Calculate net speed (magnitude of linear velocity)
                # net_speed = np.sqrt(linear_velocity ** 2)

                # # Publish net speed
                # net_speed_msg = Float32()
                # net_speed_msg.data = net_speed
                # self.net_speed_pub.publish(net_speed_msg)

                # Calculate and publish odometry
                if dt > 0:
                    dx = linear_velocity * dt * np.cos(self.pose_theta)
                    dy = linear_velocity * dt * np.sin(self.pose_theta)
                    dtheta = angular_velocity * dt

                    self.pose_x += dx
                    self.pose_y += dy
                    self.pose_theta += dtheta
                    raw_theta_msg = Float32()
                    raw_theta_msg.data = self.pose_theta
                    self.raw_theta_pub.publish(raw_theta_msg)

                    # Convert to radians
                    # theta_radians = math.radians(self.pose_theta + dtheta)

                    # Create and publish the Odometry message
                    odom_msg = Odometry()
                    odom_msg.header.stamp = current_time.to_msg()
                    odom_msg.child_frame_id = "odom"
                    odom_msg.pose.pose.position.x = self.pose_x + dx
                    odom_msg.pose.pose.position.y = self.pose_y + dy
                    x,y,z,w = quaternion_from_euler(0.0, 0.0, self.pose_theta + dtheta)
                    odom_msg.pose.pose.orientation.x = x
                    odom_msg.pose.pose.orientation.y = y
                    odom_msg.pose.pose.orientation.z = z
                    odom_msg.pose.pose.orientation.w = w
                    odom_msg.twist.twist.linear.x = linear_velocity
                    odom_msg.twist.twist.angular.z = angular_velocity
                    self.odom_pub.publish(odom_msg)

                # # Publish voltage
                # voltage_msg = Float32()
                # voltage_msg.data = float(data['v'])
                # self.raw_voltage_pub.publish(voltage_msg)
            except Exception as e:
                self.get_logger().warn(e)
                self.get_logger().warn(traceback.format_exc())


        elif data['T'] == 1002:
            try:
                imu_msg = Imu()

                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'imu_link'

                # Extract and convert IMU values
                roll = float(data['r'])
                pitch = float(data['p'])
                yaw = float(0.0)
                try:
                    yaw = data['y']
                except:
                    pass

                quaternion = quaternion_from_euler(roll, pitch, yaw)

                imu_msg.orientation.x = float(quaternion[0])
                imu_msg.orientation.y = float(quaternion[1])
                imu_msg.orientation.z = float(quaternion[2])
                imu_msg.orientation.w = float(quaternion[3])

                # Extract and assign angular velocity and linear acceleration
                imu_msg.angular_velocity.x = float(data['gx'])
                imu_msg.angular_velocity.y = float(data['gy'])
                imu_msg.angular_velocity.z = float(data['gz'])


                imu_msg.linear_acceleration.x = float(data['ax'])
                imu_msg.linear_acceleration.y = float(data['ay'])
                imu_msg.linear_acceleration.z = float(data['az'])

                self.raw_imu_pub.publish(imu_msg)

                # Apply calibration to linear acceleration
                imu_msg.linear_acceleration.x = data['ax'] * self.imu_calibration_data['x_scale'] + self.imu_calibration_data['x_offset']
                imu_msg.linear_acceleration.y = data['ay'] * self.imu_calibration_data['y_scale'] + self.imu_calibration_data['y_offset']
                imu_msg.linear_acceleration.z = data['az'] * self.imu_calibration_data['z_scale'] + self.imu_calibration_data['z_offset']
                
                self.imu_pub.publish(imu_msg)
            except Exception as e:
                self.get_logger().warn(e)
                self.get_logger().warn(traceback.format_exc())

        elif data['T'] == 1003:  # ESP-NOW received message
            try:
                # Publish the received message to /raw/msg_in
                esp_now_msg = String()
                s = data.get("mac", "") + " : " + data.get("megs", "")
                self.get_logger().info("Receiving Msg!")
                self.get_logger().info(s)
                esp_now_msg.data = s
                self.raw_esp_now_msg_pub.publish(esp_now_msg)
            except Exception as e:
                self.get_logger().warn(e)
                self.get_logger().warn(traceback.format_exc())
        else:
            print(data)

    def serial_write_json(self, data):
        msg = String()
        msg.data = json.dumps(data)
        self.serial_write_callback(msg)

    def imu_calibration_step(self, step):
        self.get_logger().warn("Imu calibration step is not implemented.")
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SerialProcessorNode("serial_processor")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()
