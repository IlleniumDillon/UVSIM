
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package='caterpillar_robot').find('caterpillar_robot')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    configuration_basename = LaunchConfiguration('configuration_basename', default='caterpillar_locating.lua')
    load_state_filename = LaunchConfiguration('load_state_filename', default=os.path.join(pkg_share, 'map/map_3024.pbstream'))
    rviz_config_path = os.path.join(pkg_share,'rviz','theworld.rviz')
    
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        namespace='robot0',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename,
                   '-load_state_filename',load_state_filename],
        #remappings=[('base_link','robot0_base_link'),('odom','robot0_odom')]
        )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
        )

    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(start_rviz_cmd)

    return ld