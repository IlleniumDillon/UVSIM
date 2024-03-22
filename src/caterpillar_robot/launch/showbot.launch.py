import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.00'

    package_name = 'caterpillar_robot'
    robot_name_in_model = 'robot'
    urdf_file_path = 'urdf/caterpillar_robot.xacro.urdf'
    #urdf_file_path = 'urdf/fishbot.urdf'
    world_file_path = 'worlds/aius3011world.world'

    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    world_path = os.path.join(pkg_share, world_file_path)  

    # doc = xacro.parse(open(default_urdf_model_path))
    # xacro.process_doc(doc)
    # params = {'robot_description': doc.toxml()}

    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[params]
    # )

    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')
    
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, 
                    '-file', default_urdf_model_path,
                        '-x', spawn_x_val,
                        '-y', spawn_y_val,
                        '-z', spawn_z_val,
                        '-Y', spawn_yaw_val],
                        output='screen')


    ld = LaunchDescription(
        [
            start_gazebo_cmd,
            spawn_entity_cmd,
            # node_robot_state_publisher
        ]
    )

    return ld