"""
R-Net bridge node (requirements 6.3, 6.4, 6.5).

Opens the SocketCAN interface directly (6.1.3). It decodes power-module status frames
into ``wheelchair_msgs/RnetStatus`` on ``/rnet/status`` at 10 Hz and mirrors received
frames on ``/from_can_bus``. When transmission is enabled it also follows the chair's
joystick module (FollowJSM, 6.3): for each centred JSM joystick frame it replies with its
own frame built from the latest ``/cmd_vel/safe``, sent straight from the receive loop to
stay within 1 ms, and mirrored on ``/to_can_bus``.

``tx_enabled`` is false by default (6.3.5), so out of the box the node only listens and is
safe to run on a powered chair. It can be toggled live with
``ros2 param set /rnet_bridge tx_enabled true``. Moving the chair's own joystick always
overrides the bridge (6.3.8). Runs the receive loop in a background thread; a lock guards
the shared status tracker.
"""

import socket
import struct
import threading

from builtin_interfaces.msg import Time
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rnet_bridge.bridge_logic import JoystickFollower, ReplyMeter, StatusTracker, TxHealth
from wheelchair_msgs.msg import RnetStatus

CAN_FRAME = struct.Struct('=IB3x8s')
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x7FF


def _to_time(seconds):
    """Convert a float time in seconds (or None) to a builtin_interfaces/Time."""
    msg = Time()
    if seconds is not None:
        msg.sec = int(seconds)
        msg.nanosec = int(round((seconds - int(seconds)) * 1e9))
    return msg


class BridgeNode(Node):
    """Reads the CAN bus in a background thread, decodes status and follows the JSM."""

    def __init__(self):
        super().__init__('rnet_bridge')
        self.declare_parameter('interface', 'can0')
        self.declare_parameter('pm_nibble', 0)
        self.declare_parameter('joystick_nibble', 2)
        self.declare_parameter('current_scale', 1.0)
        self.declare_parameter('pm_heartbeat_timeout', 2.5)
        self.declare_parameter('rx_silent_timeout', 1.0)
        self.declare_parameter('status_rate_hz', 10.0)
        self.declare_parameter('publish_can_rx', True)
        self.declare_parameter('publish_can_tx', True)
        # Transmit side (FollowJSM). tx_enabled is false by default (6.3.5).
        self.declare_parameter('tx_enabled', False)
        self.declare_parameter('linear_limit', 0.5)
        self.declare_parameter('angular_limit', 1.0)
        self.declare_parameter('max_pct', 20)
        self.declare_parameter('cmd_stale_timeout', 0.1)
        self.declare_parameter('override_rearm_time', 0.5)

        self._iface = self.get_parameter('interface').value
        self._pm_heartbeat_timeout = self.get_parameter('pm_heartbeat_timeout').value
        self._publish_can_rx = self.get_parameter('publish_can_rx').value

        self._lock = threading.Lock()
        self._tracker = StatusTracker(
            self.get_parameter('pm_nibble').value,
            current_scale=self.get_parameter('current_scale').value,
            rx_silent_timeout=self.get_parameter('rx_silent_timeout').value,
            now=self._now(),
        )
        self._follower = JoystickFollower(
            self.get_parameter('joystick_nibble').value,
            linear_limit=self.get_parameter('linear_limit').value,
            angular_limit=self.get_parameter('angular_limit').value,
            max_pct=self.get_parameter('max_pct').value,
            stale_timeout=self.get_parameter('cmd_stale_timeout').value,
            rearm_time=self.get_parameter('override_rearm_time').value,
            tx_enabled=self.get_parameter('tx_enabled').value,
        )
        self._tx_health = TxHealth()
        self._reply_meter = ReplyMeter(window=0.001)
        self._last_late = 0

        self._status_pub = self.create_publisher(RnetStatus, 'rnet/status', 10)
        self._rx_pub = (self.create_publisher(Frame, 'from_can_bus', 50)
                        if self._publish_can_rx else None)
        self._tx_pub = (self.create_publisher(Frame, 'to_can_bus', 50)
                        if self.get_parameter('publish_can_tx').value else None)
        self.create_subscription(Twist, 'cmd_vel/safe', self._on_safe, 10)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._sock = self._open_socket()
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        rate = self.get_parameter('status_rate_hz').value
        self.create_timer(1.0 / rate, self._publish_status)
        self._log_startup(rate)

    def _log_startup(self, rate):
        if self._follower.tx_enabled:
            self.get_logger().warning(
                f"rnet_bridge on '{self._iface}' with TRANSMIT ENABLED: following the JSM "
                f'(reply to centred frames). /rnet/status at {rate:.0f} Hz.')
        else:
            self.get_logger().info(
                f"rnet_bridge on '{self._iface}' receive-only (tx_enabled=false): "
                f'/rnet/status at {rate:.0f} Hz. Not transmitting.')

    def _now(self):
        """Return the current ROS time in seconds as a float."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _open_socket(self):
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
        sock.settimeout(0.5)
        sock.bind((self._iface,))
        return sock

    # --- inputs ---------------------------------------------------------------------------

    def _on_safe(self, msg):
        """Store the latest /cmd_vel/safe command for the next JSM reply (6.3.3)."""
        self._follower.set_command(msg.linear.x, msg.angular.z, self._now())

    def _on_set_parameters(self, params):
        """Allow tx_enabled to be toggled at runtime (6.3.5)."""
        for param in params:
            if param.name == 'tx_enabled':
                self._follower.tx_enabled = bool(param.value)
                state = 'ENABLED' if param.value else 'disabled'
                self.get_logger().warning(f'tx_enabled {state}')
        return SetParametersResult(successful=True)

    # --- receive loop (also sends the time-critical FollowJSM reply) ----------------------

    def _read_loop(self):
        while self._running:
            try:
                raw = self._sock.recv(16)
            except socket.timeout:
                continue
            except OSError as err:
                if self._running:
                    self.get_logger().warning(f'CAN recv error: {err}')
                continue
            now = self._now()
            raw_id, dlc, data = CAN_FRAME.unpack(raw)
            extended = bool(raw_id & CAN_EFF_FLAG)
            frame_id = raw_id & (CAN_EFF_MASK if extended else CAN_SFF_MASK)
            payload = data[:dlc]
            # FollowJSM reply first, to stay within 1 ms of the JSM frame (6.3.1).
            if self._follower.is_jsm_frame(frame_id):
                reply = self._follower.on_jsm_frame(payload, now)
                if reply is not None:
                    self._send_reply(reply[0], reply[1], now)
            with self._lock:
                self._tracker.on_frame(frame_id, payload, now)
            if self._rx_pub is not None:
                self._rx_pub.publish(self._to_can_msg(raw_id, dlc, data, extended))

    def _send_reply(self, frame_id, payload, jsm_now):
        packed = CAN_FRAME.pack(frame_id | CAN_EFF_FLAG, len(payload),
                                bytes(payload).ljust(8, b'\x00'))
        try:
            self._sock.send(packed)
        except OSError as err:
            self._tx_health.record(False, jsm_now)
            self.get_logger().warning(f'CAN send error: {err}')
            return
        self._tx_health.record(True, jsm_now)
        self._reply_meter.record(jsm_now, self._now())
        if self._tx_pub is not None:
            self._tx_pub.publish(
                self._to_can_msg(frame_id | CAN_EFF_FLAG, len(payload),
                                 bytes(payload).ljust(8, b'\x00'), True))

    @staticmethod
    def _to_can_msg(raw_id, dlc, data, extended):
        msg = Frame()
        msg.id = raw_id & (CAN_EFF_MASK if extended else CAN_SFF_MASK)
        msg.is_extended = extended
        msg.is_rtr = bool(raw_id & CAN_RTR_FLAG)
        msg.is_error = bool(raw_id & CAN_ERR_FLAG)
        msg.dlc = dlc
        msg.data = list(data)
        return msg

    # --- outputs --------------------------------------------------------------------------

    def _publish_status(self):
        now = self._now()
        with self._lock:
            fields = dict(self._tracker.fields)
            stamps = dict(self._tracker.stamps)
            rx_count = self._tracker.rx_count
            unknown = sum(self._tracker.unknown.values())
            rx_silent = self._tracker.rx_silent(now)

        msg = RnetStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        pm_seen = stamps.get('pm_alive')
        msg.pm_alive = pm_seen is not None and (now - pm_seen) <= self._pm_heartbeat_timeout
        msg.pm_last_seen = _to_time(pm_seen)
        msg.battery_pct = int(fields.get('battery_pct', 0))
        msg.battery_last_updated = _to_time(stamps.get('battery_pct'))
        msg.motor_current_a = float(fields.get('motor_current_a', 0.0))
        msg.motor_current_last_updated = _to_time(stamps.get('motor_current_a'))
        msg.motors_stopped = bool(fields.get('motors_stopped', False))
        msg.motors_decelerating = bool(fields.get('motors_decelerating', False))
        msg.rx_frame_count = rx_count
        msg.unknown_frame_count = unknown
        msg.rx_silent = rx_silent
        self._status_pub.publish(msg)
        self._report_reply_health()

    def _report_reply_health(self):
        """Warn once per window when replies miss the 1 ms budget (6.3.7)."""
        if self._reply_meter.late > self._last_late:
            self.get_logger().warning(
                f'{self._reply_meter.late} JSM replies over 1 ms '
                f'(max {self._reply_meter.max_delay * 1e3:.2f} ms of '
                f'{self._reply_meter.replies} replies)')
        self._last_late = self._reply_meter.late

    def destroy_node(self):
        self._running = False
        if self._reader.is_alive():
            self._reader.join(timeout=1.0)
        try:
            self._sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
