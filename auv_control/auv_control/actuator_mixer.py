
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool, Header
from auv_interfaces.msg import ActuatorCommand


class ActuatorMixer(Node):
    def __init__(self):
        super().__init__('actuator_mixer')

        # --- Parameters ------------------------------------------------------
        self.declare_parameter('source_timeout_s', 0.3)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('fin_pitch_limit_rad', 0.5)
        self.declare_parameter('fin_yaw_limit_rad', 0.5)

        self._timeout_s = self.get_parameter('source_timeout_s').value
        rate = self.get_parameter('publish_rate_hz').value
        self._pitch_limit = self.get_parameter('fin_pitch_limit_rad').value
        self._yaw_limit = self.get_parameter('fin_yaw_limit_rad').value

        # --- QoS -------------------------------------------------------------
        # MUST match the supervisor's /estop publisher: reliable +
        # transient-local. A volatile subscriber will not connect to a
        # transient-local publisher — the e-stop would silently never arrive.
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        cmd_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # --- State: latest command per source, with arrival time -------------
        # We store (command, arrival_time). arrival_time is OUR clock, same
        # reasoning as the watchdog: staleness is measured locally, immune to
        # any source's clock skew.
        self._estopped = True                 # fail-safe default
        self._safety_cmd = None
        self._teleop = None                   # (ActuatorCommand, Time)
        self._autonomy = None                 # (ActuatorCommand, Time)

        # --- Subscriptions ---------------------------------------------------
        self.create_subscription(Bool, '/estop', self._on_estop, latched_qos)
        self.create_subscription(
            ActuatorCommand, '/cmd/safety', self._on_safety, cmd_qos)
        self.create_subscription(
            ActuatorCommand, '/cmd/teleop', self._on_teleop, cmd_qos)
        self.create_subscription(
            ActuatorCommand, '/cmd/autonomy', self._on_autonomy, cmd_qos)

        # --- Publisher -------------------------------------------------------
        self._pub = self.create_publisher(
            ActuatorCommand, '/cmd/actuators', cmd_qos)

        # --- Arbitration timer -----------------------------------------------
        self._timer = self.create_timer(1.0 / rate, self._arbitrate)
        self.get_logger().info(
            f'Actuator mixer up. timeout={self._timeout_s}s '
            f'pitch_lim={self._pitch_limit} yaw_lim={self._yaw_limit}')

    # ------------------------------------------------------------------ #
    #  Input callbacks: just record latest + arrival time                #
    # ------------------------------------------------------------------ #
    def _on_estop(self, msg: Bool):
        self._estopped = msg.data

    def _on_safety(self, msg: ActuatorCommand):
        self._safety_cmd = msg

    def _on_teleop(self, msg: ActuatorCommand):
        self._teleop = (msg, self.get_clock().now())

    def _on_autonomy(self, msg: ActuatorCommand):
        self._autonomy = (msg, self.get_clock().now())

    # ------------------------------------------------------------------ #
    #  Arbitration                                                        #
    # ------------------------------------------------------------------ #
    def _is_live(self, entry):
        """A source is live if it has sent a command within source_timeout_s.
        None (never sent) is not live. This is what makes a crashed source's
        last command expire instead of persisting."""
        if entry is None:
            return False
        _, stamp = entry
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age <= self._timeout_s

    def _arbitrate(self):
        # Rule 1: e-stop wins unconditionally.
        if self._estopped:
            self._publish_safe()
            return

        # Rule 2: highest-priority live source. teleop > autonomy.
        if self._is_live(self._teleop):
            self._publish(self._teleop[0])
            return
        if self._is_live(self._autonomy):
            self._publish(self._autonomy[0])
            return

        # Rule 3: nothing live -> fail to safe.
        self._publish_safe()

    def _publish_safe(self):
        """Publish the supervisor's safe command if we have one; otherwise
        synthesise a fail-to-surface command locally. We never trust that the
        safety topic has arrived — if it hasn't, we still must be safe."""
        if self._safety_cmd is not None:
            self._publish(self._safety_cmd)
        else:
            cmd = ActuatorCommand()
            cmd.header = Header()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.thruster = 0.0
            cmd.fin_pitch = 0.0
            cmd.fin_yaw = 0.0
            cmd.ballast_rate = -1.0   # fail to surface
            cmd.source = ActuatorCommand.SOURCE_SAFETY
            self._publish(cmd)

    def _publish(self, cmd: ActuatorCommand):
        """Clamp to mechanical limits, restamp, publish. Clamping happens HERE
        because this is the last stage that understands normalised intent
        before the driver converts to PWM."""
        out = ActuatorCommand()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.thruster = self._clamp(cmd.thruster, -1.0, 1.0)
        out.fin_pitch = self._clamp(cmd.fin_pitch, -self._pitch_limit, self._pitch_limit)
        out.fin_yaw = self._clamp(cmd.fin_yaw, -self._yaw_limit, self._yaw_limit)
        out.ballast_rate = self._clamp(cmd.ballast_rate, -1.0, 1.0)
        out.source = cmd.source   # preserve who won, for bag analysis
        self._pub.publish(out)

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()