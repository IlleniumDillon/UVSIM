from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'True')

    ## ***** File paths ******
    pkg_share = FindPackageShare(package='caterpillar_robot').find('caterpillar_robot')

    # 配置文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    # 配置文件
    configuration_basename = LaunchConfiguration('configuration_basename', default='caterpillar_locating.lua')

    statefile = os.path.join(
        pkg_share,'map/mapforsim.pbstream'
    )
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        arguments = [
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename,
            '-load_state_filename',statefile],
        remappings = [
            ('scan', 'scan')],
        output = 'screen'
        )
    
    rviz_node = Node(
        package='rviz2',
        namespace='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen')
    return LaunchDescription([
        use_sim_time_arg,
        rviz_node,
        cartographer_node,
    ])

