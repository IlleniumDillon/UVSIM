
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, TextSubstitution

def gen_robot_list(number_of_robots):

    robots = []

    for i in range(number_of_robots):
        robot_name = "robot"+str(i)
        x_pos = float(i)
        robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': 0.0, 'z_pose': 0.25})


    return robots 

def generate_launch_description():
    robot_name_in_model = 'mycar'
    package_name = 'caterpillar_robot'
    urdf_name = "caterpillar_robot.xacro"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    gazebo_world_path = os.path.join(pkg_share, 'worlds/aius3011world.world')

    # Start Gazebo server
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        output='screen')
    ld.add_action(start_gazebo_cmd)

    robots = gen_robot_list(5)

    spawn_robots_cmds = []
    robots_state_publisher_cmds = []
    for robot in robots:
        ld.add_action(
            ExecuteProcess(
                cmd=[[
                    'xacro ',urdf_model_path,
                    ' -o ',os.path.join(pkg_share, 'urdf/'+robot['name']+'.urdf'),
                    ' ns:=',robot['name'],
                    ' robot_name:=',robot['name']
                ]],
                shell=True
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
    # Launch the robot
    # spawn_entity_cmd = Node(
    #     package='gazebo_ros', 
    #     executable='spawn_entity.py',
    #     arguments=['-entity', robot_name_in_model,  '-file', urdf_model_path ], output='screen')
	
    # # Start Robot State publisher
    # start_robot_state_publisher_cmd = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     arguments=[urdf_model_path]
    # )

    return ld