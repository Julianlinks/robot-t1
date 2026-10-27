import os
from pathlib import Path
import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler, EmitEvent)
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('sort_place_sim'))
    gz_share = Path(get_package_share_directory('ros_gz_sim'))
    description = xacro.process_file(str(share / 'urdf/scara.urdf.xacro'),
        mappings={'controllers': str(share / 'config/controllers.yaml')}).toxml()
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'sort_cell', '-name', 'scara',
                            '-topic', '/robot_description', '-allow_renaming', 'false'])
    broadcaster = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster', '--controller-manager-timeout', '90'])
    arm = Node(package='controller_manager', executable='spawner', output='screen',
               arguments=['arm_controller', '--controller-manager-timeout', '90'])
    task = Node(package='sort_place_core', executable='task_server', output='screen',
                parameters=[{'use_sim_time': True, 'config_file': str(share / 'config/cell.json')}])

    def after_success(next_action):
        def callback(event, context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='Cell startup process failed'))]
            return [next_action]
        return callback

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gz_args', default_value='-r -v 3'),
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',
                                  os.path.join(get_package_prefix('sort_place_sim'), 'lib')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz_share / 'launch/gz_sim.launch.py')),
                                 launch_arguments={'gz_args': [LaunchConfiguration('gz_args'),
                                                              ' ', str(share / 'worlds/cell.sdf')]}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'),
        # Register all handlers before starting the process chain.
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_success(broadcaster))),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=after_success(arm))),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=after_success(task))),
        spawn,
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'rviz/cell.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz')))
    ])
