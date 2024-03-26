import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    mapfile = os.path.join(
        get_package_share_directory('uvplanner'),'map/map_2.yaml'
    )
    mapserver =     Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False}, 
                        {'yaml_filename':mapfile}])
    lifecycle = Node(package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[{'use_sim_time': False},
                                {'autostart': True},
                                {'node_names': ['map_server']}])
    return LaunchDescription([
        mapserver,
        lifecycle,
    ])