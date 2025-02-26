import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument

# Function to get the localplan config file
def get_localplan_config_file(context):
    # Get the use_localplan launch configuration
    use_localplan = LaunchConfiguration('use_localplan').perform(context)

    # Get the ugv_nav package share directory
    ugv_nav_dir = get_package_share_directory('ugv_nav')

    # Get the paths to the teb and dwa param files
    teb_param_path = os.path.join(ugv_nav_dir, 'param', 'rtabmap_teb.yaml')
    dwa_param_path = os.path.join(ugv_nav_dir, 'param', 'rtabmap_dwa.yaml')

    # Create a dictionary to map the use_localplan configuration to the corresponding param file
    config_map = {
        'teb': teb_param_path,
        'dwa': dwa_param_path
    }

    # Return the corresponding param file based on the use_localplan configuration
    return config_map.get(use_localplan, teb_param_path)

# Function to set up the launch description
def launch_setup(context, *args, **kwargs):

    # Get the localplan config file
    param_file = get_localplan_config_file(context)
    # Get the ugv_nav and nav2_bringup package share directories
    ugv_nav_dir = get_package_share_directory('ugv_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ������������
    # Get the map yaml path
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(ugv_nav_dir, 'maps', 'map.yaml'))
                                                         
    # Include the nav2 bringup launch description
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_path,
            'params_file': param_file
        }.items()
    )
    
    # Declare the use_rviz launch argument
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false',
                                     description='Whether to launch RViz2')  

    # Include the bringup lidar launch description
    bringup_lidar_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ugv_bringup'), 'launch'),
         '/bringup_lidar.launch.py']),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': 'nav_3d',
        }.items()
    )
                                         
    # Include the robot pose publisher launch description
    robot_pose_publisher_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('robot_pose_publisher'), 'launch'),
         '/robot_pose_publisher_launch.py'])
    ) 
        
    # Return the launch actions
    return [
        use_rviz_arg,
        bringup_lidar_launch,
        robot_pose_publisher_launch,
        nav2_bringup_launch
    ]

# Function to generate the launch description
def generate_launch_description():

    # Return the launch description
    return LaunchDescription([
        DeclareLaunchArgument('use_localplan', default_value='dwa', description='Choose which localplan to use: dwa, teb'),
        OpaqueFunction(function=launch_setup)
    ])

# Main function
if __name__ == '__main__':
    generate_launch_description()
