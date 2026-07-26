"""Actuator driver: converts /cmd/actuators into MAVLink RC override.

The ONLY node that commands hardware. Uses /mavros/rc/override (a TOPIC,
mavros_msgs/OverrideRCIn) rather than the command service, because on this
Pixhawk 1 + ArduSub setup the mavros command plugin fails to initialise
(AUTOPILOT_VERSION unsupported), leaving /mavros/cmd/command with no live
server. RC override does not depend on that plugin. See ADR-011.

REQUIRES on the FCU: SYSID_MYGCS = 255, or ArduSub ignores the override.

Channel map (RC input channels, 1-indexed in the vehicle, 0-indexed in the
array): thruster=ch1[0], pitch=ch2[1], yaw=ch3[2]. A value of 0 (or 65535)
means 'no override' for that channel.

Defense in depth: also subscribes to /estop and forces neutral when stopped.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)

from std_msgs.msg import Bool
from auv_interfaces.msg import ActuatorCommand
from mavros_msgs.msg import OverrideRCIn

NO_CHANGE = 0          # 0 tells ArduPilot "leave this channel alone"
NUM_CHANNELS = 18      # OverrideRCIn.channels is length 18


class ActuatorDriver(Node):
    def __init__(self):
        super().__init__('actuator_driver')

        # Channel indices in the 18-element array (0-based).
        self.declare_parameter('thruster_ch_index', 0)   # RC1
        self.declare_parameter('pitch_ch_index', 1)      # RC2
        self.declare_parameter('yaw_ch_index', 2)        # RC3

        self.declare_parameter('pwm_neutral', 1500)
        self.declare_parameter('pwm_min', 1100)
        self.declare_parameter('pwm_max', 1900)
        self.declare_parameter('thruster_scale', 400.0)
        self.declare_parameter('fin_rad_to_pwm', 400.0)
        self.declare_parameter('pitch_sign', 1)
        self.declare_parameter('yaw_sign', 1)
        self.declare_parameter('fin_pwm_span_limit', 200)
        self.declare_parameter('publish_rate_hz', 20.0)

        self._i_thr = self.get_parameter('thruster_ch_index').value
        self._i_pitch = self.get_parameter('pitch_ch_index').value
        self._i_yaw = self.get_parameter('yaw_ch_index').value
        self._neutral = self.get_parameter('pwm_neutral').value
        self._pmin = self.get_parameter('pwm_min').value
        self._pmax = self.get_parameter('pwm_max').value
        self._thr_scale = self.get_parameter('thruster_scale').value
        self._fin_scale = self.get_parameter('fin_rad_to_pwm').value
        self._pitch_sign = self.get_parameter('pitch_sign').value
        self._yaw_sign = self.get_parameter('yaw_sign').value
        self._fin_span = self.get_parameter('fin_pwm_span_limit').value
        rate = self.get_parameter('publish_rate_hz').value

        self._estopped = True
        # Latest command, held so the timer can republish at a steady rate.
        # RC override must be sent continuously or ArduPilot times it out.
        self._latest = self._neutral_channels()

        latched_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(Bool, '/estop', self._on_estop, latched_qos)

        cmd_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(
            ActuatorCommand, '/cmd/actuators', self._on_cmd, cmd_qos)

        self._pub = self.create_publisher(
            OverrideRCIn, '/mavros/rc/override', 10)

        # Republish at fixed rate. ArduPilot expects a continuous override
        # stream; if it stops, the FCU reverts to its own RC failsafe.
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            'Actuator driver up (RC override mode). '
            'Requires SYSID_MYGCS=255 on FCU.')

    def _neutral_channels(self):
        ch = [NO_CHANGE] * NUM_CHANNELS
        ch[self._i_thr] = self._neutral
        ch[self._i_pitch] = self._neutral
        ch[self._i_yaw] = self._neutral
        return ch

    def _on_estop(self, msg: Bool):
        self._estopped = msg.data
        if self._estopped:
            self._latest = self._neutral_channels()

    def _on_cmd(self, msg: ActuatorCommand):
        if self._estopped:
            self._latest = self._neutral_channels()
            return
        ch = [NO_CHANGE] * NUM_CHANNELS
        ch[self._i_thr] = self._clamp(
            int(self._neutral + msg.thruster * self._thr_scale),
            self._pmin, self._pmax)
        pitch_off = self._clamp(
            int(msg.fin_pitch * self._fin_scale * self._pitch_sign),
            -self._fin_span, self._fin_span)
        yaw_off = self._clamp(
            int(msg.fin_yaw * self._fin_scale * self._yaw_sign),
            -self._fin_span, self._fin_span)
        ch[self._i_pitch] = self._neutral + pitch_off
        ch[self._i_yaw] = self._neutral + yaw_off
        self._latest = ch

    def _publish(self):
        out = OverrideRCIn()
        out.channels = self._latest
        self._pub.publish(out)

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()