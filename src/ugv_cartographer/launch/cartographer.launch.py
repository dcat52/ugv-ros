import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Positioning function pack
    pkg_share = FindPackageShare(package='ugv02_cartographer').find('ugv02_cartographer')
    
    # Configure node launch information 
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Map resolution
    resolution = LaunchConfiguration('resolution', default='0.05')
    # Map publish period  
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # Configuration file folder path
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    # Configuration file
    configuration_basename = LaunchConfiguration('configuration_basename', default='cartographer_2d.lua')

    #Nodes
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename])

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    #Launch file
    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)

    return ld
