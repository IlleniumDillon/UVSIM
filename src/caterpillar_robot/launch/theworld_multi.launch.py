
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


def gen_robot_list(number_of_robots):

    robots = []

    for i in range(number_of_robots):
        robot_name = "robot"+str(i)
        x_pos = i*0.5
        color = "Green"
        namespace = robot_name
        robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': 0.0, 'z_pose': 0.25,
                       'color': color, 'namespace': namespace})


    return robots 

# def gen_robot_list():
#     robots = []
#     for i in range(10):
#         robot_name = "red_group1_"+str(i)
#         x_pos = int(i/7)*0.5 - 10
#         y_pos = float(i%7)*0.5
#         color = "Red"
#         namespace = "redg1"
#         robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': y_pos, 'z_pose': 0.25,
#                        'color': color, 'namespace': namespace})
    
#     for i in range(15):
#         robot_name = "red_group2_"+str(i)
#         x_pos = int(i/7)*0.5 - 10
#         y_pos = float(i%7)*0.5 - 10
#         color = "Red"
#         namespace = "redg2"
#         robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': y_pos, 'z_pose': 0.25,
#                        'color': color, 'namespace': namespace})
        
#     for i in range(10):
#         robot_name = "blu_group1_"+str(i)
#         x_pos = int(i/7)*0.5 + 10
#         y_pos = float(i%7)*0.5
#         color = "Blue"
#         namespace = "blug1"
#         robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': y_pos, 'z_pose': 0.25,
#                        'color': color, 'namespace': namespace})
    
#     for i in range(15):
#         robot_name = "blu_group2_"+str(i)
#         x_pos = int(i/7)*0.5 + 10
#         y_pos = float(i%7)*0.5 + 10
#         color = "Blue"
#         namespace = "blug2"
#         robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': y_pos, 'z_pose': 0.25,
#                        'color': color, 'namespace': namespace})
#     return robots 

def generate_launch_description():
    robot_name_in_model = 'mycar'
    package_name = 'caterpillar_robot'
    urdf_name = "caterpillar_robot.xacro"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    gazebo_world_path = os.path.join(pkg_share, 'worlds/aius3011world.world')
    rviz_config_path = os.path.join(pkg_share,'rviz','theworld.rviz')
    mapfile = os.path.join(
        get_package_share_directory('caterpillar_robot'),'map/mapforsim.yaml'
    )

    # Start Gazebo server
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        # cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen')
    ld.add_action(start_gazebo_cmd)

    robots = gen_robot_list(1)

    spawn_robots_cmds = []
    robots_state_publisher_cmds = []
    for robot in robots:
        ld.add_action(
            ExecuteProcess(
                cmd=[[
                    'xacro ',urdf_model_path,
                    ' -o ',os.path.join(pkg_share, 'urdf/'+robot['name']+'.urdf'),
                    ' robot_name:=',robot['name'],
                    ' robot_color:=',robot['color'],
                    ' robot_namespace:=',robot['namespace']
                ]],
                shell=True
            )
        )
        ld.add_action(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments = ['0', '0', '0', 
                             '0', '0', '0', 
                             'map', robot['name']+'_odom']
            )
        )
        ld.add_action(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                namespace='/'+robot['name'],
                name='spawm'+robot['name'],
                arguments=[
                    '-entity', robot['name'],
                    '-file', os.path.join(pkg_share, 'urdf/'+robot['name']+'.urdf'),
                    '-x', TextSubstitution(text=str(robot['x_pose'])),
                    '-y', TextSubstitution(text=str(robot['y_pose'])),
                    '-z', TextSubstitution(text=str(robot['z_pose'])),
                ],
                output='screen'
            )
        )
        ld.add_action(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='/'+robot['name'],
                name='state'+robot['name'],
                arguments=[os.path.join(pkg_share, 'urdf/'+robot['name']+'.urdf')]
            )
        )
        # ld.add_action(
        #     Node(
        #         package='simplanner',
        #         executable='uvplanner_node',
        #         namespace='/'+robot['name'],
        #         parameters=[
        #             {"robot_name": robot['name']},
        #             {"map_file": mapfile}
        #         ]
        #     )
        # )
    # start_rviz_cmd = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_path]
    #     )
    # ld.add_action(start_rviz_cmd)

    return ld