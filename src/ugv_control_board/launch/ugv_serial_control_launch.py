from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    serial_processor_node = Node(
        package='ugv_control_board',
        executable='serial_processor',
        output='screen',
        emulate_tty=True  # To see log messages in the console
    )

    velocity_command_node = Node(
        package='ugv_control_board',
        executable='velocity_command',
        output='screen',
        emulate_tty=True
    )

    msg_out_node = Node(
        package='ugv_control_board',
        executable='msg_out',
        output='screen',
        emulate_tty=True
    )


    tf_odom_to_base = Node(package = "tf2_ros", 
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0", "0", "0", "0", "odom", "base_link"])

    tf_base_to_mount = Node(package = "tf2_ros", 
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0.18", "0", "0", "0", "base_link", "mount_link"])

    tf_mount_to_imu = Node(package = "tf2_ros", 
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0", "0", "0", "0", "mount_link", "imu_link"])

    tf_mount_to_laser = Node(package = "tf2_ros", 
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0", "1.57", "0", "0", "mount_link", "laser_link"])
                


    tf_odom_to_odom_frame = Node(package = "tf2_ros", 
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0", "0", "0", "0", "odom", "odom_frame"])

    tf_footprint_to_base = Node(package = "tf2_ros", 
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"])


    return LaunchDescription([
        tf_odom_to_base,
        serial_processor_node,
        velocity_command_node,
        msg_out_node,
        tf_base_to_mount,
        tf_mount_to_imu,
        tf_mount_to_laser,
        tf_odom_to_odom_frame,
        tf_footprint_to_base
    ])