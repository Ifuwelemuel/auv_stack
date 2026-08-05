"""PCA9685 actuator driver: the single node that commands hardware.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from smbus2 import SMBus

from std_msgs.msg import Bool
from auv_interfaces.msg import ActuatorCommand

# PCA9685 registers
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
# MODE1 bits
RESTART, SLEEP, AI, ALLCALL = 0x80, 0x10, 0x20, 0x01
# MODE2 bits
OUTDRV = 0x04


class PCA9685:
    """Minimal PCA9685 driver over smbus2, with read-back verification."""

    def __init__(self, bus_num, addr, freq_hz=50):
        self._bus = SMBus(bus_num)
        self._addr = addr
        self._freq = freq_hz
        self.init()

    # --- low-level register access -------------------------------------- #
    def _w(self, reg, val):
        self._bus.write_byte_data(self._addr, reg, val)

    def _r(self, reg):
        return self._bus.read_byte_data(self._addr, reg)

    # --- configuration --------------------------------------------------- #
    def init(self):
        """Totem-pole outputs, wake from sleep, set frequency.
        PRESCALE can only be written while the chip is asleep, hence the
        sleep / write / restore sequence."""
        self._w(MODE2, OUTDRV)
        self._w(MODE1, ALLCALL)
        time.sleep(0.005)
        self._w(MODE1, self._r(MODE1) & ~SLEEP)      # wake
        time.sleep(0.005)
        prescale = int(round(25_000_000.0 / (4096.0 * self._freq)) - 1)
        old = self._r(MODE1)
        self._w(MODE1, (old & 0x7F) | SLEEP)          # sleep to set prescale
        self._w(PRESCALE, prescale)
        self._w(MODE1, old)                            # restore (wakes)
        time.sleep(0.005)
        self._w(MODE1, old | RESTART | AI)             # restart + auto-inc

    def verify(self):
        """Read back PRESCALE and the SLEEP bit to confirm the chip actually
        took our configuration. Returns (ok, got, want, asleep).

        This exists because a failed init is SILENT: the chip stays asleep at
        its default frequency, every subsequent write appears to succeed, and
        the actuators simply never move."""
        want = int(round(25_000_000.0 / (4096.0 * self._freq)) - 1)
        try:
            got = self._r(PRESCALE)
            mode1 = self._r(MODE1)
        except OSError:
            return False, -1, want, True
        asleep = bool(mode1 & SLEEP)
        return (got == want and not asleep), got, want, asleep

    # --- output ----------------------------------------------------------- #
    def set_us(self, channel, microseconds):
        """Set one channel's pulse width in microseconds."""
        ticks = int(4096 * microseconds / (1_000_000.0 / self._freq))
        ticks = max(0, min(4095, ticks))
        base = LED0_ON_L + 4 * channel
        self._w(base + 0, 0)                    # ON_L
        self._w(base + 1, 0)                    # ON_H
        self._w(base + 2, ticks & 0xFF)         # OFF_L
        self._w(base + 3, (ticks >> 8) & 0x0F)  # OFF_H

    def close(self):
        self._bus.close()


class PCA9685Driver(Node):

    def __init__(self):
        super().__init__('pca9685_driver')

        # --- I2C / board -------------------------------------------------- #
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_addr', 0x40)

        # --- Channel map (ADR-012) --------------------------------------- #
        self.declare_parameter('pitch_channel', 0)
        self.declare_parameter('yaw_channel', 1)
        self.declare_parameter('thruster_channel', 3)

        # --- Calibrated fin ranges, PER AXIS (ADR-013) ------------------- #
        # Each servo has its own linkage geometry and mechanical stops, so
        # neutral and span are measured and stored separately. Do NOT assume
        # the two axes match. Commanding past a stop stalls the servo, which
        # draws full current and can strip gears.
        self.declare_parameter('pitch_centre_us', 825)
        self.declare_parameter('pitch_span_us', 325)
        self.declare_parameter('yaw_centre_us', 825)
        self.declare_parameter('yaw_span_us', 325)
        self.declare_parameter('fin_max_rad', 0.5)

        # Direction signs describe the HARDWARE, not operator preference.
        # Stick-direction inversions belong in the teleop layer, so that
        # autonomous controllers issuing the same commands still get the
        # physically correct deflection.
        self.declare_parameter('pitch_sign', 1)
        self.declare_parameter('yaw_sign', 1)

        # --- Thruster / ESC ---------------------------------------------- #
        # Marine ESC convention: neutral = stop, bidirectional. Neutral is
        # MEASURED (ADR-016) — the nominal 1500us read as a small throttle on
        # this board, preventing arming and causing the motor to creep.
        self.declare_parameter('thruster_stop_us', 1480)
        self.declare_parameter('thruster_span_us', 200)
        self.declare_parameter('esc_arm_hold_s', 2.0)

        p = self.get_parameter
        self._bus_num = p('i2c_bus').value
        self._addr = p('i2c_addr').value
        self._ch_pitch = p('pitch_channel').value
        self._ch_yaw = p('yaw_channel').value
        self._ch_thr = p('thruster_channel').value
        self._pitch_centre = p('pitch_centre_us').value
        self._pitch_span = p('pitch_span_us').value
        self._yaw_centre = p('yaw_centre_us').value
        self._yaw_span = p('yaw_span_us').value
        self._fin_max_rad = p('fin_max_rad').value
        self._pitch_sign = p('pitch_sign').value
        self._yaw_sign = p('yaw_sign').value
        self._thr_stop = p('thruster_stop_us').value
        self._thr_span = p('thruster_span_us').value
        self._arm_hold = p('esc_arm_hold_s').value

        # --- Hardware bring-up ------------------------------------------- #
        try:
            self._pca = PCA9685(self._bus_num, self._addr)
            self.get_logger().info(
                f'PCA9685 opened on i2c-{self._bus_num} @ 0x{self._addr:02x}')
        except Exception as e:
            self.get_logger().error(f'PCA9685 open failed: {e}')
            raise

        # Verify the configuration actually took, retrying if not. Without
        # this, a silent init failure produces a node that logs "ready" while
        # the actuators are dead.
        configured = False
        for attempt in range(1, 6):
            ok, got, want, asleep = self._pca.verify()
            if ok:
                self.get_logger().info(
                    f'PCA9685 verified: prescale={got}, awake.')
                configured = True
                break
            self.get_logger().warn(
                f'PCA9685 not configured (prescale got={got} want={want}, '
                f'asleep={asleep}) — re-initialising, attempt {attempt}/5')
            time.sleep(0.2)
            try:
                self._pca.init()
            except OSError as e:
                self.get_logger().warn(f'  re-init raised: {e}')
        if not configured:
            self.get_logger().error(
                'PCA9685 FAILED to configure after 5 attempts — actuators '
                'will NOT respond. Check I2C wiring, board power and bus.')

        # --- Safe state before anything else ----------------------------- #
        self._estopped = True            # fail-safe default
        self._apply_safe()

        # ESC arming: hold the stop signal so the ESC arms before we accept
        # throttle. Subscriptions are created AFTER this, so no command can
        # arrive mid-arm. Note the ESC arms only at ITS power-up, so it must
        # be powered while this signal is already streaming.
        self.get_logger().info(
            f'Holding thruster stop ({self._thr_stop}us) for '
            f'{self._arm_hold}s (ESC arming)...')
        time.sleep(self._arm_hold)
        self.get_logger().info('ESC arming window complete.')

        # --- QoS ---------------------------------------------------------- #
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        cmd_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST)

        self.create_subscription(Bool, '/estop', self._on_estop, latched)
        self.create_subscription(
            ActuatorCommand, '/cmd/actuators', self._on_cmd, cmd_qos)

        self.get_logger().info(
            f'PCA9685 driver ready. Booted SAFE. '
            f'pitch={self._pitch_centre}±{self._pitch_span}us, '
            f'yaw={self._yaw_centre}±{self._yaw_span}us, '
            f'thruster stop={self._thr_stop}us')

    # ------------------------------------------------------------------ #
    #  Callbacks                                                          #
    # ------------------------------------------------------------------ #
    def _on_estop(self, msg: Bool):
        self._estopped = msg.data
        if self._estopped:
            self._apply_safe()

    def _on_cmd(self, msg: ActuatorCommand):
        if self._estopped:
            self._apply_safe()
            return

        pitch_us = self._fin_to_us(
            msg.fin_pitch, self._pitch_centre, self._pitch_span,
            self._pitch_sign)
        yaw_us = self._fin_to_us(
            msg.fin_yaw, self._yaw_centre, self._yaw_span, self._yaw_sign)
        thr = max(-1.0, min(1.0, msg.thruster))
        thr_us = int(self._thr_stop + thr * self._thr_span)

        try:
            self._pca.set_us(self._ch_pitch, pitch_us)
            self._pca.set_us(self._ch_yaw, yaw_us)
            self._pca.set_us(self._ch_thr, thr_us)
        except OSError as e:
            self.get_logger().error(f'I2C write failed: {e}')

    # ------------------------------------------------------------------ #
    #  Helpers                                                            #
    # ------------------------------------------------------------------ #
    def _fin_to_us(self, angle_rad, centre_us, span_us, sign):
        """Radians -> microseconds for ONE fin, using that fin's own
        calibrated centre and span. The fraction is clamped to [-1, 1] first,
        so the output can never exceed centre ± span and therefore can never
        drive the servo past its measured mechanical limits."""
        frac = max(-1.0, min(1.0, angle_rad / self._fin_max_rad))
        return centre_us + int(frac * span_us * sign)

    def _apply_safe(self):
        """Thruster stopped, each fin at ITS OWN calibrated neutral.
        Using a shared neutral would leave one servo off-centre in the safe
        state. (Ballast safe state is owned by a separate node — ADR-004.)"""
        try:
            self._pca.set_us(self._ch_thr, self._thr_stop)
            self._pca.set_us(self._ch_pitch, self._pitch_centre)
            self._pca.set_us(self._ch_yaw, self._yaw_centre)
        except OSError as e:
            self.get_logger().error(f'I2C write failed applying safe: {e}')

    def destroy_node(self):
        try:
            self._apply_safe()
            self._pca.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PCA9685Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()