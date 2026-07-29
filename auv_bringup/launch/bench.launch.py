"""Full bench stack: safety, control, teleop, PCA9685 actuation, mavros bridge,
depth sensing, heading/depth control, LOS guidance, waypoint missions.

    ros2 launch auv_bringup bench.launch.py                    # normal bench run
    ros2 launch auv_bringup bench.launch.py record:=true       # + rosbag dataset
    ros2 launch auv_bringup bench.launch.py heading:=true      # + heading controller
    ros2 launch auv_bringup bench.launch.py depth:=true        # + Bar02 depth node
    ros2 launch auv_bringup bench.launch.py depth_ctrl:=true   # + depth controller
    ros2 launch auv_bringup bench.launch.py guidance:=true     # + LOS guidance
    ros2 launch auv_bringup bench.launch.py mission:=/path/to/mission.yaml
    ros2 launch auv_bringup bench.launch.py use_local_joy:=false  # joy on buoy Pi
    ros2 launch auv_bringup bench.launch.py mavros:=false      # actuation-only work

If the graph already has stack nodes running, this launch REFUSES to start
(the ghost guard). Run scripts/kill_stack.sh first — or use
scripts/clean_launch.sh, which purges deliberately and then launches.
"""
import subprocess
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    #
    
    ghosts = subprocess.run(['pgrep', '-af', 'install/auv_'],
                            capture_output=True, text=True).stdout.strip()
    if ghosts:
        raise RuntimeError(
            '\n\nGHOST NODES already running — refusing to stack a second '
            'launch.\nRun scripts/kill_stack.sh (or use '
            'scripts/clean_launch.sh), then relaunch.\nFound:\n' + ghosts + '\n')

    pkg_control = FindPackageShare('auv_control')
    pkg_bringup = FindPackageShare('auv_bringup')

    # --- Launch arguments ---------------------------------------------------
    # Every DeclareLaunchArgument
    
    use_local_joy = DeclareLaunchArgument(
        'use_local_joy', default_value='true',
        description='Run joy_node on this machine. Set false only when joy '
                    'comes from the buoy Pi.')

    record = DeclareLaunchArgument(
        'record', default_value='false',
        description='Record a rosbag of this session. Opt-in: bags are '
                    'datasets, not by-products.')

    heading = DeclareLaunchArgument(
        'heading', default_value='false',
        description='Run the heading controller (autonomy composer, ADR-027). '
                    'Clearing e-stop with this on commands cruise thrust.')

    depth = DeclareLaunchArgument(
        'depth', default_value='false',
        description='Run the Bar02 depth sensor node. Requires the sensor '
                    'physically wired (flag and copper travel together).')

    depth_ctrl = DeclareLaunchArgument(
        'depth_ctrl', default_value='false',
        description='Run the depth controller (publishes /cmd/pitch; the '
                    'heading controller composes it into /cmd/autonomy).')

    guidance = DeclareLaunchArgument(
        'guidance', default_value='false',
        description='Run LOS guidance (GPS + target -> /cmd/heading_setpoint; '
                    'owns the local frame and publishes /guidance/position).')

    mission = DeclareLaunchArgument(
        'mission', default_value='',
        description='Path to a mission YAML. Empty = no waypoint manager. '
                    'Non-empty = the manager launches with that mission.')

    mavros = DeclareLaunchArgument(
        'mavros', default_value='true',
        description='Include the mavros sensor bridge. Set false only for '
                    'actuation-only bench work.')

    # --- Nodes ----------------------------------------------------------------
    safety = Node(package='auv_safety', executable='safety_supervisor',
                  name='safety_supervisor', output='screen')

    mixer = Node(package='auv_control', executable='actuator_mixer',
                 name='actuator_mixer', output='screen',
                 parameters=[PathJoinSubstitution(
                     [pkg_control, 'config', 'mixer_params.yaml'])])

    joy = Node(package='joy', executable='joy_node', name='joy_node',
               output='screen',
               
               parameters=[{'autorepeat_rate': 20.0}],
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

    depth_node = Node(package='auv_control', executable='depth_sensor',
                      name='depth_sensor', output='screen',
                      parameters=[PathJoinSubstitution(
                          [pkg_control, 'config', 'depth_params.yaml'])],
                      condition=IfCondition(LaunchConfiguration('depth')))

    depth_controller = Node(package='auv_control', executable='depth_controller',
                            name='depth_controller', output='screen',
                            parameters=[PathJoinSubstitution(
                                [pkg_control, 'config', 'depth_ctrl_params.yaml'])],
                            condition=IfCondition(LaunchConfiguration('depth_ctrl')))

    los_guidance = Node(package='auv_control', executable='los_guidance',
                        name='los_guidance', output='screen',
                        parameters=[PathJoinSubstitution(
                            [pkg_control, 'config', 'los_params.yaml'])],
                        condition=IfCondition(LaunchConfiguration('guidance')))

    waypoint_manager = Node(
        package='auv_control', executable='waypoint_manager',
        name='waypoint_manager', output='screen',
        parameters=[
            PathJoinSubstitution(
                [pkg_control, 'config', 'waypoint_params.yaml']),
            # Layered params, last writer wins: the YAML's empty
            # mission_file is overridden by the launch argument's path.
            {'mission_file': LaunchConfiguration('mission')},
        ],
        # Conditioning on the argument's VALUE: assemble the Python
        # expression  '<path>' != ''  — true iff a mission was given.
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('mission'), "' != ''"])))

    # --- mavros: composed, one command owns the system ------------------------
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [pkg_bringup, 'launch', 'mavros.launch.py'])),
        condition=IfCondition(LaunchConfiguration('mavros')))

    
    bag_name = f'bags/bench_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_name,
             '/joy', '/cmd/teleop', '/cmd/safety', '/cmd/autonomy',
             '/cmd/actuators', '/cmd/heading_setpoint', '/estop', '/heartbeat',
             '/mavros/mavros/data', '/depth', '/pressure',
             '/cmd/pitch', '/cmd/depth_setpoint',
             '/buoy/fix', '/guidance/target', '/guidance/position'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record')))

    return LaunchDescription([
        # arguments
        use_local_joy, record, heading, depth, depth_ctrl, guidance,
        mission, mavros,
        # nodes 
        safety, mixer, joy, teleop, driver, heading_ctrl,
        depth_node, depth_controller, los_guidance, waypoint_manager,
        mavros_launch,
        recorder,
    ])