#!/usr/bin/env python3
# serial_processor_node.py
import numpy as np

import json

import serial

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

from std_msgs.msg import String, Float32, Int32MultiArray
from sensor_msgs.msg import Imu, BatteryState

from tf_transformations import quaternion_from_euler

wheel_distance = 0.08  # meters
wheel_radius = 0.05  # meters

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
            return  # Exit if serial port cannot be opened

        self.local_commands_sub = self.create_subscription(String, 'raw/local_cmds', self.raw_command_callback, 10)

        # Create a subscriber to receive commands
        self.serial_write_sub = self.create_subscription(
            String, 'raw/serial_out', self.serial_write_callback, 1000)

        # Create a timer to periodically read serial data
        self.serial_read_timer = self.create_timer(0.001, self.read_serial_data)

        # Create a timer to periodically request IMU data
        self.imu_request_timer = self.create_timer(0.1, self.request_imu_data) 

        # Create publishers for different data types
        self.raw_serial_msg_pub = self.create_publisher(String, 'raw/serial_in', 1000)
        self.raw_left_wheel_vel_pub = self.create_publisher(Float32, 'raw/left_wheel_velocity', 40)
        self.raw_right_wheel_vel_pub = self.create_publisher(Float32, 'raw/right_wheel_velocity', 40)
        self.raw_roll_pitch_pub = self.create_publisher(Int32MultiArray, 'raw/roll_pitch', 40)
        self.raw_voltage_pub = self.create_publisher(Float32, 'raw/battery_voltage', 40)
        self.raw_imu_pub = self.create_publisher(Imu, 'raw/imu_data', 40)
        self.raw_esp_now_msg_pub = self.create_publisher(String, 'raw/msg_in', 40)
        self.imu_pub = self.create_publisher(Imu, 'imu', 40)

        self.disable_feedback_flow()


        # TODO(Gemini)
        # Create variables for calibration status
        # Create variables for storing imu data
        self.imu_calibration_data = {
            'x_scale': 1.004, 'y_scale': 1.006, 'z_scale': 0.977, 
            'x_offset': 0.0, 'y_offset': 0.0, 'z_offset': 0.0, 
        }
        self.imu_prior_step = -1
        self.imu_msg_buffer = []  # Buffer to store IMU data during calibration
        self.calibrating_imu = False

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
            self.get_logger().warn(f"Failed to parse serial data: {e}")
        except Exception as e:  # Catch any other exceptions
            self.get_logger().warn(line)
            self.get_logger().warn(f"Failed to read or decode serial data: {e}") 

    def serial_write_callback(self, msg):
        # Get the command string from the message
        command = msg.data

        # Send the command to the serial port
        try:
            self.ser.write(command.encode('utf-8') + b'\n')
            # self.get_logger().info("Command sent successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def raw_command_callback(self, msg):
        data = json.loads(msg.data)

        if data['T'] == 127:
            self.imu_calibration_step(data['step'])

        elif data['T'] == 128:
            print(data['msg'])

    def process_serial_data(self, data):

        if data['T'] == 1001:
            # Publish left wheel velocity
            left_wheel_msg = Float32()
            left_wheel_msg.data = float(data['L'])
            self.raw_left_wheel_vel_pub.publish(left_wheel_msg)

            # Publish right wheel velocity
            right_wheel_msg = Float32()
            right_wheel_msg.data = float(data['R'])
            self.raw_right_wheel_vel_pub.publish(right_wheel_msg)

            # Publish IMU data
            intarr_msg = Int32MultiArray()
            arr = []
            arr.append(int(data['r']))  # Convert 'r' to an integer
            arr.append(int(data['p']))  # Convert 'p' to an integer
            intarr_msg.data = arr  # Assign the array to the 'data' field of the message
            self.raw_roll_pitch_pub.publish(intarr_msg)

            # Publish voltage
            voltage_msg = Float32()
            voltage_msg.data = float(data['v'])
            self.raw_voltage_pub.publish(voltage_msg)

        elif data['T'] == 1002:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            # Extract and convert IMU values
            roll = data['r']
            pitch = data['p']
            yaw = 0.0

            quaternion = quaternion_from_euler(roll, pitch, yaw)

            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]

            # Extract and assign angular velocity and linear acceleration
            imu_msg.angular_velocity.x = data['gx']
            imu_msg.angular_velocity.y = data['gy']
            imu_msg.angular_velocity.z = data['gz']


            imu_msg.linear_acceleration.x = data['ax']
            imu_msg.linear_acceleration.y = data['ay']
            imu_msg.linear_acceleration.z = data['az']

            if self.calibrating_imu:
                # TODO(Gemini)
                # append imu data to array
                self.imu_msg_buffer.append(imu_msg)
                pass

            self.raw_imu_pub.publish(imu_msg)

            # Apply calibration to linear acceleration
            imu_msg.linear_acceleration.x = data['ax'] * self.imu_calibration_data['x_scale'] + self.imu_calibration_data['x_offset']
            imu_msg.linear_acceleration.y = data['ay'] * self.imu_calibration_data['y_scale'] + self.imu_calibration_data['y_offset']
            imu_msg.linear_acceleration.z = data['az'] * self.imu_calibration_data['z_scale'] + self.imu_calibration_data['z_offset']
            
            self.imu_pub.publish(imu_msg)

        elif data['T'] == 1003:  # ESP-NOW received message
                # Publish the received message to /raw/msg_in
                esp_now_msg = String()
                s = data.get("mac", "") + " : " + data.get("megs", "")
                print(s)
                esp_now_msg.data = s
                self.raw_esp_now_msg_pub.publish(esp_now_msg)
        else:
            print(data)

    def imu_calibration_step(self, step):

        if self.calibrating_imu is False and self.imu_prior_step >= 0:
            # TODO(Gemini)
            # call process serial data T=128 and msg=... Fatal Error, invalid imu calibration state (e1)
            self.process_serial_data({"T": 128, "msg": "Fatal Error, invalid imu calibration state (e1)"})
            return

        if self.calibrating_imu is True and step != self.imu_prior_step+1:
            # TODO(Gemini)
            # call process serial data T=128 and msg=... Warning, incorrect calibration sequence, ingoring
            self.process_serial_data({"T": 128, "msg": "Warning, incorrect calibration sequence, ignoring"})
            return
        
        self.calibrating_imu = True

        cutoff_time = self.get_clock().now() - Time(seconds=2.0, clock_type=self.get_clock().clock_type)
        cutoff_time_obj = self.get_clock().now() - cutoff_time  # Convert to Time object

        imu_data = self.imu_msg_buffer.copy()
        imu_data = [img_msg.linear_acceleration for img_msg in imu_data if Time.from_msg(img_msg.header.stamp) > cutoff_time_obj]  # Assuming data['timestamp'] is a ROS 2 Time message

        print(f'{len(imu_data)} imu msgs for calibration step')

        if step == 0:
            self.process_serial_data({"T": 128, "msg": "Instruction (x-axis up)"})  # Call process_serial_data
            self.imu_prior_step = step
            self.imu_calibration_data['x_offset'] = 0.0
            self.imu_calibration_data['y_offset'] = 0.0
            self.imu_calibration_data['z_offset'] = 0.0
            self.imu_calibration_data['x_scale'] = 1.0
            self.imu_calibration_data['y_scale'] = 1.0
            self.imu_calibration_data['z_scale'] = 1.0

        # Compute necessary calibration values
        if step == 1:
            # X-axis calibration (x scale)
            acc_x = np.mean([data.x for data in imu_data])
            self.imu_calibration_data['u_x_up'] = acc_x
            self.process_serial_data({"T": 128, "msg": "Instruction (y-axis up)"})
            self.imu_prior_step = step

        elif step == 2:
            # Y-axis accel (y scale)
            acc_y = np.mean([data.y for data in imu_data])
            self.imu_calibration_data['u_y_up'] = acc_y

            # Z-axis accel (z offset)
            acc_z = np.mean([data.z for data in imu_data])
            self.imu_calibration_data['u_z_side'] = acc_z

            self.process_serial_data({"T": 128, "msg": "Instruction (z-axis up)"})
            self.imu_prior_step = step

        elif step == 3:
            # X and Y-axis accel (x,y offset)
            acc_x = np.mean([data.x for data in imu_data])
            self.imu_calibration_data['u_x_side'] = acc_x
            acc_y = np.mean([data.y for data in imu_data])
            self.imu_calibration_data['u_y_side'] = acc_y

            # Z-axis accel (z scale)
            acc_z = np.mean([data.z for data in imu_data])
            self.imu_calibration_data['u_z_up'] = acc_z

            self.imu_calibration_data["x_offset"] = 0.0 - self.imu_calibration_data['u_x_side']
            self.imu_calibration_data["y_offset"] = 0.0 - self.imu_calibration_data['u_y_side']
            self.imu_calibration_data["z_offset"] = 0.0 - self.imu_calibration_data['u_z_side']

            self.imu_calibration_data["x_scale"] = 9.8 / (self.imu_calibration_data['u_x_up'] + self.imu_calibration_data['u_x_side'])
            self.imu_calibration_data["y_scale"] = 9.8 / (self.imu_calibration_data['u_y_up'] + self.imu_calibration_data['u_y_side'])
            self.imu_calibration_data["z_scale"] = 9.8 / (self.imu_calibration_data['u_z_up'] + self.imu_calibration_data['u_z_side'])

            self.process_serial_data({"T": 128, "msg": f"Final calibration values: {self.imu_calibration_data}"})

            self.calibrating_imu = False  # Reset calibrating flag
            self.imu_prior_step = -1

        imu_data = None
        self.imu_msg_buffer = []

        # if step == 0:
        #     # TODO(Gemini)
        #     # call process serial data T=128 and msg=... Intruction (x-axis up)
        #     pass
        # if step == 1:
        #     # TODO(Gemini)
        #     # get current time stamp
        #     # copy imu calibration data array. filter collected imu msgs to be the most recent 2 seconds. 
        #     # Compute necessary calibration values (offsets and scalars for x,y,z)
        #     # clear calibration data array
        #     # call process serial data T=128 and msg=... Intruction (y-axis up)
        #     pass
        # if step == 2:
        #     # TODO(Gemini)
        #     # get current time stamp
        #     # copy imu calibration data array. filter collected imu msgs to be the most recent 2 seconds. 
        #     # Compute necessary calibration values (offsets and scalars for x,y,z)
        #     # clear calibration data array
        #     # call process serial data T=128 and msg=... Intruction (z-axis up)
        #     pass
        # if step == 3:
        #     # TODO(Gemini)
        #     # get current time stamp
        #     # copy imu calibration data array. filter collected imu msgs to be the most recent 2 seconds. 
        #     # Compute necessary calibration values (offsets and scalars for x,y,z)
        #     # clear calibration data array
        #     # compute aggregate calibation values
        #     # call process serial data T=128 and msg=... Final calibration values
        #     pass



    def request_imu_data(self):
        """Requests IMU data from the UGV."""

        # Create the JSON message for requesting IMU data
        data = {"T": 126}

        imu_request_msg = String()
        imu_request_msg.data = json.dumps(data)

        # Call the serial_write_callback directly
        self.serial_write_callback(imu_request_msg) 

    def disable_feedback_flow(self):
        # Create the JSON message
        data = {"T": 131, "cmd": 0}

        # Create a ROS 2 String message
        msg = String()
        msg.data = json.dumps(data)

        # Call the serial_write_callback function
        self.serial_write_callback(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialProcessorNode("serial_processor")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    # rclpy.spin(node)

    rclpy.shutdown()
