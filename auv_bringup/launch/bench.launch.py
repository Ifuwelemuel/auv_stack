"""Full bench stack: safety, control, teleop, PCA9685 actuation, mavros bridge.

    ros2 launch auv_bringup bench.launch.py                    # normal bench run
    ros2 launch auv_bringup bench.launch.py record:=true       # + rosbag dataset
    ros2 launch auv_bringup bench.launch.py heading:=true      # + heading controller
    ros2 launch auv_bringup bench.launch.py use_local_joy:=false  # joy on buoy Pi
    ros2 launch auv_bringup bench.launch.py mavros:=false      # actuation-only work
"""
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_control = FindPackageShare('auv_control')
    pkg_bringup = FindPackageShare('auv_bringup')

    #  Launch arguments 
    t.
    use_local_joy = DeclareLaunchArgument(
        'use_local_joy', default_value='true',
        description='Run joy_node on this machine. Default TRUE: the normal '
                    'bench case is the gamepad on the vehicle computer. Set '
                    'false only when joy comes from the buoy Pi.')

    record = DeclareLaunchArgument(
        'record', default_value='false',
        description='Record a rosbag of this session. Opt-in: bags are '
                    'datasets, not by-products.')

    heading = DeclareLaunchArgument(
        'heading', default_value='false',
        description='Run the heading controller. Opt-in until the loop is '
                    'tuned: clearing e-stop with this on commands cruise '
                    'thrust immediately.')

    mavros = DeclareLaunchArgument(
        'mavros', default_value='true',
        description='Include the mavros sensor bridge. Phase 2+ needs IMU; '
                    'set false only for actuation-only bench work.')

    # Nodes 
    safety = Node(package='auv_safety', executable='safety_supervisor',
                  name='safety_supervisor', output='screen')

    mixer = Node(package='auv_control', executable='actuator_mixer',
                 name='actuator_mixer', output='screen',
                 parameters=[PathJoinSubstitution(
                     [pkg_control, 'config', 'mixer_params.yaml'])])

    joy = Node(package='joy', executable='joy_node', name='joy_node',
               output='screen',
               condition=IfCondition(LaunchConfiguration('use_local_joy')))

    teleop = Node(package='auv_control', executable='teleop_node',
                  name='teleop_node', output='screen',
                  parameters=[PathJoinSubstitution(
                      [pkg_control, 'config', 'teleop_params.yaml'])])

    driver = Node(package='auv_control', executable='pca9685_driver',
                  name='pca9685_driver', output='screen',
                  parameters=[PathJoinSubstitution(
                      [pkg_control, 'config', 'pca9685_params.yaml'])])

    heading_ctrl = Node(package='auv_control', executable='heading_controller',
                        name='heading_controller', output='screen',
                        parameters=[PathJoinSubstitution(
                            [pkg_control, 'config', 'heading_params.yaml'])],
                        condition=IfCondition(LaunchConfiguration('heading')))

    #  mavros
    
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [pkg_bringup, 'launch', 'mavros.launch.py'])),
        condition=IfCondition(LaunchConfiguration('mavros')))

    #  bag recording 
    
    bag_name = f'bags/bench_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_name,
             '/joy', '/cmd/teleop', '/cmd/safety', '/cmd/autonomy',
             '/cmd/actuators', '/cmd/heading_setpoint', '/estop', '/heartbeat',
             '/mavros/mavros/data'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record')))

    return LaunchDescription([
        use_local_joy, record, heading, mavros,   # every argument, or it doesn't exist
        safety, mixer, joy, teleop, driver, heading_ctrl,
        mavros_launch,
        recorder,
    ])