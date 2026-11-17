"""
Receive-only R-Net bridge node (requirements 6.4, 6.5).

Opens the SocketCAN interface directly (6.1.3), decodes power-module status frames into
``wheelchair_msgs/RnetStatus`` on ``/rnet/status`` at 10 Hz, and mirrors every received
frame on ``/from_can_bus`` for logging.

This node never transmits. The FollowJSM reply path (6.3) is added in a later step. Until
then the chair cannot be driven from ROS, so this is safe to run on a powered chair. For
extra safety the interface may also be left in listen-only mode (``ip link ... listen-only
on``); this node receives either way.
"""

import socket
import struct
import threading

from builtin_interfaces.msg import Time
from can_msgs.msg import Frame
import rclpy
from rclpy.node import Node
from rnet_bridge.bridge_logic import StatusTracker
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
    """Reads the CAN bus in a background thread and publishes decoded status."""

    def __init__(self):
        super().__init__('rnet_bridge')
        self.declare_parameter('interface', 'can0')
        self.declare_parameter('pm_nibble', 0)
        self.declare_parameter('current_scale', 1.0)
        self.declare_parameter('pm_heartbeat_timeout', 2.5)
        self.declare_parameter('rx_silent_timeout', 1.0)
        self.declare_parameter('status_rate_hz', 10.0)
        self.declare_parameter('publish_can_rx', True)

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

        self._status_pub = self.create_publisher(RnetStatus, 'rnet/status', 10)
        self._rx_pub = (self.create_publisher(Frame, 'from_can_bus', 50)
                        if self._publish_can_rx else None)

        self._sock = self._open_socket()
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        rate = self.get_parameter('status_rate_hz').value
        self._timer = self.create_timer(1.0 / rate, self._publish_status)
        self.get_logger().info(
            f"rnet_bridge receive-only on '{self._iface}': decoding /rnet/status at "
            f'{rate:.0f} Hz. Not transmitting.')

    def _now(self):
        """Return the current ROS time in seconds as a float."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _open_socket(self):
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
        sock.settimeout(0.5)
        sock.bind((self._iface,))
        return sock

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
            with self._lock:
                self._tracker.on_frame(frame_id, payload, now)
            if self._rx_pub is not None:
                self._rx_pub.publish(self._to_can_msg(raw_id, dlc, data, extended))

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
