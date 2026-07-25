"""Bring up the full Phase 0 bench stack: sensors, safety, control, teleop.

    ros2 launch auv_bringup bench.launch.py

Props OFF. This launches everything except the mavros->hardware actuator
driver, which comes in Unit 0.6.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = FindPackageShare('auv_bringup')
    pkg_control = FindPackageShare('auv_control')

    mavros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [pkg_bringup, 'launch', 'mavros.launch.py'])),
    )

    safety = Node(package='auv_safety', executable='safety_supervisor',
                  name='safety_supervisor', output='screen')

    mixer = Node(package='auv_control', executable='actuator_mixer',
                 name='actuator_mixer', output='screen',
                 parameters=[PathJoinSubstitution(
                     [pkg_control, 'config', 'mixer_params.yaml'])])

    joy = Node(package='joy', executable='joy_node', name='joy_node',
               output='screen')

    teleop = Node(package='auv_control', executable='teleop_node',
                  name='teleop_node', output='screen',
                  parameters=[PathJoinSubstitution(
                      [pkg_control, 'config', 'teleop_params.yaml'])])

    return LaunchDescription([mavros, safety, mixer, joy, teleop])