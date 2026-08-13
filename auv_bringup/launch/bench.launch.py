"""Full Phase 0 bench stack: safety, control, teleop, PCA9685 actuation.

    ros2 launch auv_bringup bench.launch.py


"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_control = FindPackageShare('auv_control')

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

    driver = Node(package='auv_control', executable='pca9685_driver',
                  name='pca9685_driver', output='screen',
                  parameters=[PathJoinSubstitution(
                      [pkg_control, 'config', 'pca9685_params.yaml'])])

    return LaunchDescription([safety, mixer, joy, teleop, driver])
