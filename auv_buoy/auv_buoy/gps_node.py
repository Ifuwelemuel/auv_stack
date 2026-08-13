"""Surface buoy GPS node.

Published:
    /buoy/fix        sensor_msgs/NavSatFix
    /buoy/gps_status diagnostic string
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import serial
import pynmea2

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String


class BuoyGpsNode(Node):

    def __init__(self):
        super().__init__('buoy_gps')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('frame_id', 'gps_buoy')
        # Nominal horizontal accuracy in metres at HDOP=1. Covariance scales
        # as (base * HDOP)^2. Refine once static scatter is measured on open
        # water — this figure feeds the training loss weighting directly.
        self.declare_parameter('base_accuracy_m', 2.5)
        self.declare_parameter('status_every', 10)

        p = self.get_parameter
        self._port = p('port').value
        self._baud = p('baud').value
        self._frame = p('frame_id').value
        self._base_acc = p('base_accuracy_m').value
        self._status_every = p('status_every').value

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST)

        self._fix_pub = self.create_publisher(NavSatFix, '/buoy/fix', qos)
        self._status_pub = self.create_publisher(
            String, '/buoy/gps_status', qos)

        # GSV (satellites in view) and GGA (position) arrive as separate
        # sentences, so the satellite count is cached between them.
        self._sats_in_view = 0
        self._fix_count = 0

        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=1.0)
            self.get_logger().info(
                f'GPS open on {self._port} @ {self._baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {self._port}: {e}')
            raise

        # Poll in a timer rather than a blocking loop so the node stays
        # responsive to shutdown. 20 Hz keeps up with a 1 Hz fix rate.
        self._timer = self.create_timer(0.05, self._read_serial)

    # ------------------------------------------------------------------ #
    def _read_serial(self):
        try:
            raw = self._ser.readline().decode('ascii', errors='ignore').strip()
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read failed: {e}')
            return
        if not raw.startswith('$'):
            return
        try:
            msg = pynmea2.parse(raw)
        except pynmea2.ParseError:
            return          # NMEA streams contain occasional malformed lines

        stype = getattr(msg, 'sentence_type', None)
        if stype == 'GSV':
            try:
                self._sats_in_view = int(msg.num_sv_in_view)
            except (TypeError, ValueError):
                pass
        elif stype == 'GGA':
            self._handle_gga(msg)

    # ------------------------------------------------------------------ #
    def _handle_gga(self, msg):
        """GGA carries position, fix quality, satellites used and HDOP."""
        try:
            quality = int(msg.gps_qual)
        except (TypeError, ValueError):
            quality = 0

        fix = NavSatFix()
        # Stamp at ARRIVAL on this machine's clock — see module docstring.
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self._frame

        if quality == 0:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
        elif quality == 2:
            fix.status.status = NavSatStatus.STATUS_SBAS_FIX
        else:
            fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = (NavSatStatus.SERVICE_GPS
                              | NavSatStatus.SERVICE_GLONASS)

        if quality > 0 and msg.latitude and msg.longitude:
            fix.latitude = float(msg.latitude)
            fix.longitude = float(msg.longitude)
            fix.altitude = float(msg.altitude) if msg.altitude else 0.0
        else:
            # Publish even without a fix so the ABSENCE of a fix is recorded
            # rather than appearing as an unexplained gap. Downstream code
            # must check status, not assume every message has a position.
            fix.latitude = float('nan')
            fix.longitude = float('nan')
            fix.altitude = float('nan')

        try:
            hdop = float(msg.horizontal_dil)
        except (TypeError, ValueError):
            hdop = 99.99

        if quality == 0 or hdop >= 99.0:
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            fix.position_covariance = [0.0] * 9
        else:
            sigma_h = self._base_acc * hdop
            sigma_v = sigma_h * 2.0     # vertical is typically ~2x worse
            fix.position_covariance = [
                sigma_h ** 2, 0.0, 0.0,
                0.0, sigma_h ** 2, 0.0,
                0.0, 0.0, sigma_v ** 2,
            ]
            fix.position_covariance_type = \
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self._fix_pub.publish(fix)

        self._fix_count += 1
        if self._fix_count % self._status_every == 0:
            try:
                used = int(msg.num_sats)
            except (TypeError, ValueError):
                used = 0
            s = String()
            s.data = (f'quality={quality} used={used} '
                      f'in_view={self._sats_in_view} hdop={hdop:.2f}')
            self._status_pub.publish(s)
            self.get_logger().info(s.data)

    def destroy_node(self):
        try:
            self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BuoyGpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
