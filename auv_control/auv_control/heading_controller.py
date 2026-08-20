"""Heading controller AND autonomy command composer (ADR-027).

IMU topic is a parameter: it has moved twice (ADR-009; mavros 2026.6).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32, Header
from sensor_msgs.msg import Imu
from auv_interfaces.msg import ActuatorCommand


def yaw_from_quat(q) -> float:
    """ENU yaw from quaternion (REP-103). ZYX extraction, yaw only."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(angle: float) -> float:
    """Map any angle to (-pi, pi] — always turn the short way round."""
    return math.atan2(math.sin(angle), math.cos(angle))


class HeadingController(Node):
    def __init__(self):
        super().__init__('heading_controller')

        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('i_limit', 0.2)
        self.declare_parameter('cruise_thrust', 0.3)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('imu_timeout_s', 0.5)
        self.declare_parameter('imu_topic', '/mavros/mavros/data')
        # Auxiliary channel staleness: a silent depth loop zeroes pitch.
        self.declare_parameter('pitch_timeout_s', 0.5)

        p = self.get_parameter
        self._kp, self._ki, self._kd = p('kp').value, p('ki').value, p('kd').value
        self._i_lim = p('i_limit').value
        self._cruise = p('cruise_thrust').value
        self._imu_timeout = p('imu_timeout_s').value
        self._pitch_timeout = p('pitch_timeout_s').value
        rate = p('rate_hz').value

        self._yaw = None
        self._setpoint = None
        self._last_imu_time = None
        self._integral = 0.0
        self._prev_error = None
        self._dt = 1.0 / rate

        # Auxiliary pitch channel from the depth controller (ADR-027).
        self._pitch_cmd = 0.0
        self._last_pitch_time = None

        self.create_subscription(
            Imu, p('imu_topic').value, self._on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Float32, '/cmd/heading_setpoint', self._on_setpoint, 10)
        self.create_subscription(
            Float32, '/cmd/pitch', self._on_pitch, 10)

        self._cmd_pub = self.create_publisher(ActuatorCommand, '/cmd/autonomy', 10)
        self._timer = self.create_timer(self._dt, self._on_timer)

        self.get_logger().info(
            f'Heading controller up (autonomy composer, ADR-027). '
            f'kp={self._kp} ki={self._ki} kd={self._kd} cruise={self._cruise} '
            f'imu={p("imu_topic").value}. Waiting for IMU and setpoint.')

    def _on_imu(self, msg: Imu):
        self._yaw = yaw_from_quat(msg.orientation)
        self._last_imu_time = self.get_clock().now()

    def _on_setpoint(self, msg: Float32):
        self._setpoint = wrap(msg.data)
        self._integral = 0.0
        self._prev_error = None

    def _on_pitch(self, msg: Float32):
        self._pitch_cmd = msg.data
        self._last_pitch_time = self.get_clock().now()

    def _fresh_pitch(self) -> float:
        """Zero a stale pitch channel — dead loops leave neutral fins."""
        if self._last_pitch_time is None:
            return 0.0
        age = (self.get_clock().now() - self._last_pitch_time).nanoseconds * 1e-9
        return self._pitch_cmd if age <= self._pitch_timeout else 0.0

    def _on_timer(self):
        if self._yaw is None or self._setpoint is None:
            return
        age = (self.get_clock().now() - self._last_imu_time).nanoseconds * 1e-9
        if age > self._imu_timeout:
            self.get_logger().warn('IMU stale — controller muted.',
                                   throttle_duration_sec=2.0)
            return

        error = wrap(self._setpoint - self._yaw)
        self._integral += error * self._dt
        self._integral = max(-self._i_lim, min(self._i_lim, self._integral))
        deriv = 0.0 if self._prev_error is None else \
            wrap(error - self._prev_error) / self._dt
        self._prev_error = error

        fin = self._kp * error + self._ki * self._integral + self._kd * deriv

        cmd = ActuatorCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.source = ActuatorCommand.SOURCE_AUTONOMY
        cmd.thruster = self._cruise
        cmd.fin_yaw = fin
        cmd.fin_pitch = self._fresh_pitch()     # depth loop's channel (ADR-027)
        cmd.ballast_rate = 0.0
        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()