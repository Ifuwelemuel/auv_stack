"""Bring up the mavros bridge to the Pixhawk running ArduSub.

Usage:
    ros2 launch auv_bringup mavros.launch.py
    ros2 launch auv_bringup mavros.launch.py fcu_url:=/dev/ttyACM0:115200

The fcu_url argument is the connection string to the flight controller.
Default targets the udev symlink we set up so the port is stable across reboots.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Launch arguments: everything that changes between bench and lake ----
    # fcu_url is the FC connection. On the bench over USB it's a serial device.
    # We default to the udev symlink; fall back to /dev/ttyACM0 if not yet set.
    fcu_url = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/pixhawk:115200',   # udev symlink, ADR-022 (supersedes ADR-008)
        description='Flight controller connection URL (device:baud)',
    )

    # gcs_url lets a ground station (QGC on your Mac) connect THROUGH mavros
    # via UDP, so QGC and ROS can both see the Pixhawk at once. Empty = off.
    gcs_url = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='Ground station forwarding URL, e.g. udp://@<mac-ip>:14550',
    )

    params_file = PathJoinSubstitution([
        FindPackageShare('auv_bringup'), 'config', 'mavros_params.yaml',
    ])

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        # namespace removed — the node name already gives us the /mavros prefix
        output='screen',
        parameters=[
            params_file,
            {'fcu_url': LaunchConfiguration('fcu_url')},
            {'gcs_url': LaunchConfiguration('gcs_url')},
        ],
    )

    return LaunchDescription([fcu_url, gcs_url, mavros_node])