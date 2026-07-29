
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
RESTART, SLEEP, AI, ALLCALL, OUTDRV = 0x80, 0x10, 0x20, 0x01, 0x04


class PCA9685:
    """Minimal PCA9685 driver over smbus2. Proven in bench calibration."""
    def __init__(self, bus_num, addr, freq_hz=50):
        self._bus = SMBus(bus_num)
        self._addr = addr
        self._freq = freq_hz
        self._init()

    def _w(self, reg, val):
        self._bus.write_byte_data(self._addr, reg, val)

    def _r(self, reg):
        return self._bus.read_byte_data(self._addr, reg)

    def _init(self):
        self._w(MODE2, OUTDRV)
        self._w(MODE1, ALLCALL)
        time.sleep(0.005)
        self._w(MODE1, self._r(MODE1) & ~SLEEP)
        time.sleep(0.005)
        prescale = int(round(25_000_000.0 / (4096.0 * self._freq)) - 1)
        old = self._r(MODE1)
        self._w(MODE1, (old & 0x7F) | SLEEP)
        self._w(PRESCALE, prescale)
        self._w(MODE1, old)
        time.sleep(0.005)
        self._w(MODE1, old | RESTART | AI)

    def set_us(self, channel, microseconds):
        ticks = int(4096 * microseconds / (1_000_000.0 / self._freq))
        ticks = max(0, min(4095, ticks))
        base = LED0_ON_L + 4 * channel
        self._w(base + 0, 0)
        self._w(base + 1, 0)
        self._w(base + 2, ticks & 0xFF)
        self._w(base + 3, (ticks >> 8) & 0x0F)

    def close(self):
        self._bus.close()


class PCA9685Driver(Node):
    def __init__(self):
        super().__init__('pca9685_driver')

        # --- I2C / board params ---------------------------------------------
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_addr', 0x40)

        # --- Channel map (ADR-012) ------------------------------------------
        self.declare_parameter('pitch_channel', 0)
        self.declare_parameter('yaw_channel', 1)
        self.declare_parameter('thruster_channel', 2)

        # --- Calibrated ranges (ADR-013). Fins: centre 900, symmetric +/-400
        #     (the smaller half of the 500-1500 measured range, kept
        #     symmetric for predictable control). --------------------------
        self.declare_parameter('fin_centre_us', 900)
        self.declare_parameter('fin_span_us', 400)      # +/- from centre
        # Fin command is in radians; this is the max |rad| we map to full span.
        self.declare_parameter('fin_max_rad', 0.5)

        # Direction signs (measured): pitch 1500us = nose up = positive. So
        # positive fin_pitch -> higher us -> +1. Confirm yaw separately.
        self.declare_parameter('pitch_sign', -1)
        self.declare_parameter('yaw_sign', -1)

        # --- Thruster (ESC) params. ESC neutral/stop and range. Standard ESC
        #     is 1500 stop, 1100-1900. CONFIRM your ESC before spinning. ----
        self.declare_parameter('thruster_stop_us', 1500)
        self.declare_parameter('thruster_span_us', 400)  # +/- from stop

        p = self.get_parameter
        self._bus_num = p('i2c_bus').value
        self._addr = p('i2c_addr').value
        self._ch_pitch = p('pitch_channel').value
        self._ch_yaw = p('yaw_channel').value
        self._ch_thr = p('thruster_channel').value
        self._fin_centre = p('fin_centre_us').value
        self._fin_span = p('fin_span_us').value
        self._fin_max_rad = p('fin_max_rad').value
        self._pitch_sign = p('pitch_sign').value
        self._yaw_sign = p('yaw_sign').value
        self._thr_stop = p('thruster_stop_us').value
        self._thr_span = p('thruster_span_us').value

        # --- Hardware ------------------------------------------------------
        try:
            self._pca = PCA9685(self._bus_num, self._addr)
            self.get_logger().info(
                f'PCA9685 up on i2c-{self._bus_num} @ 0x{self._addr:02x}')
        except Exception as e:
            self.get_logger().error(f'PCA9685 init failed: {e}')
            raise

        self._estopped = True   # fail-safe default
        self._apply_safe()      # start in safe state immediately

        self.get_logger().info('Holding thruster stop for ESC arming...')
        time.sleep(2.0)
        self.get_logger().info('ESC arming window complete.')
        
        # --- QoS -----------------------------------------------------------
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)
        cmd = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST)

        self.create_subscription(Bool, '/estop', self._on_estop, latched)
        self.create_subscription(
            ActuatorCommand, '/cmd/actuators', self._on_cmd, cmd)

        self.get_logger().info('PCA9685 driver ready. Booted SAFE.')

    # -------------------------------------------------------------------- #
    def _on_estop(self, msg: Bool):
        self._estopped = msg.data
        if self._estopped:
            self._apply_safe()

    def _on_cmd(self, msg: ActuatorCommand):
        if self._estopped:
            self._apply_safe()
            return
        # Fins: radians -> us, clamped to +/- span, with direction sign.
        pitch_us = self._fin_to_us(msg.fin_pitch, self._pitch_sign)
        yaw_us = self._fin_to_us(msg.fin_yaw, self._yaw_sign)
        # Thruster: normalised [-1,1] -> us around stop.
        thr = max(-1.0, min(1.0, msg.thruster))
        thr_us = int(self._thr_stop + thr * self._thr_span)

        self._pca.set_us(self._ch_pitch, pitch_us)
        self._pca.set_us(self._ch_yaw, yaw_us)
        self._pca.set_us(self._ch_thr, thr_us)

    def _fin_to_us(self, angle_rad, sign):
        frac = max(-1.0, min(1.0, angle_rad / self._fin_max_rad))
        offset = int(frac * self._fin_span * sign)
        return self._fin_centre + offset

    def _apply_safe(self):
        """Thruster stopped, fins centred. (Ballast safe is a separate node.)"""
        self._pca.set_us(self._ch_thr, self._thr_stop)
        self._pca.set_us(self._ch_pitch, self._fin_centre)
        self._pca.set_us(self._ch_yaw, self._fin_centre)

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