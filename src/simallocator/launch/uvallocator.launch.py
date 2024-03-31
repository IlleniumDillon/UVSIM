import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)

def generate_launch_description():
    mapfile = os.path.join(
        get_package_share_directory('caterpillar_robot'),'map/mapforsim.yaml'
    )
    ld = LaunchDescription()
    ld.add_action(
        Node(
            package='simallocator',
            executable='uvallocator_node',
            parameters=[
                {"map_file": mapfile},
                {"robot_list":['robot0','robot1','robot2','robot3','robot4']}
            ]
        )
    )
    return ld