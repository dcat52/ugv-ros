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

    return LaunchDescription([
        serial_processor_node,
        velocity_command_node
    ])