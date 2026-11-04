# 6. CAN Communication

Translates between ROS and the R-Net CAN bus. Packages: `rnet_bridge`, `ros2_socketcan`; Linux SocketCAN.
Input: `/cmd_vel/safe`, `/from_can_bus`. Output: `/to_can_bus`, `/rnet/status`.

Frame IDs come from the open-rnet frame dictionary
(https://github.com/redragonx/open-rnet/blob/main/reference/RNET_FRAME_DICTIONARY.md).
In each ID, `X` is the chair's device nibble.

## 6.1 Bus interface

**6.1.1** The system shall connect to the R-Net bus as a SocketCAN interface at 125 kbit/s. — P1 · T

**6.1.2** The interface name shall be a launch argument, defaulting to `can0`, so that `vcan0` can be used for testing without hardware. — P1 · I

**6.1.3** The system shall send and receive frames through `ros2_socketcan`, on `/to_can_bus` and `/from_can_bus` (`can_msgs/Frame`). — P1 · T

**6.1.4** The system shall support 29-bit extended frame IDs. — P1 · T

## 6.2 Device identification

**6.2.1** The device nibble `X` shall be a parameter. — P1 · I

**6.2.2** The correct value of `X` for this chair shall be found from a bus capture and recorded in the bringup config. — P1 · T · *TBD*

**6.2.3** How the chair's own joystick module is handled (disabled, disconnected or impersonated) shall be decided and documented before the first powered drive. Follow the open-rnet approach. — P1 · A · *TBD*

## 6.3 Command transmission

**6.3.1** `rnet_bridge` shall send the joystick frame `02000X00` (extended ID, DLC 2) every 10 ms (±2 ms) while transmission is enabled. — P1 · T

**6.3.2** Joystick frame data shall be two signed 8-bit values in the range −100 to +100:

- **6.3.2.1** Byte 0 (X): turn, from `angular.z` scaled by the angular limit.
- **6.3.2.2** Byte 1 (Y): forward/back, from `linear.x` scaled by the linear limit.
- **6.3.2.3** Values outside the range shall be clamped, never wrapped.

— P1 · T

**6.3.3** The bridge shall always send the most recent `/cmd_vel/safe` value. The send rate shall not depend on when commands arrive. — P1 · T

**6.3.4** If no `/cmd_vel/safe` message has arrived for 100 ms *(param)*, the bridge shall send X = 0, Y = 0. — P1 · T

**6.3.5** A `tx_enabled` parameter, `false` by default, shall disable all transmission. With it disabled, the bridge decodes and logs only. — P1 · T

**6.3.6** The maximum joystick percent shall be a parameter, `max_pct`, defaulting to 20. — P1 · I

**6.3.7** The bridge should measure its own send-interval jitter and report it to Diagnostics (7.2.3). — P2 · T

## 6.4 Status reception

**6.4.1** `rnet_bridge` shall decode these frames into `wheelchair_msgs/RnetStatus` on `/rnet/status`:

| Frame | Rate | Field |
|---|---|---|
| `0C140X00` | 1 s | `pm_alive`, time last seen |
| `14300X00` | 200 ms | `motor_current_a` (16-bit little-endian, scaled to amps) |
| `1C0C0X00` | 1 s | `battery_pct` (0–100) |
| `0C000005` / `0C000006` | on event | `motors_stopped` / decelerating |

— P1 · T

**6.4.2** `/rnet/status` shall be published at 10 Hz *(param)*, with the time each field was last updated. — P1 · T

**6.4.3** The bridge shall count frames it can't decode by ID, and shall not raise an error for them. — P2 · T

**6.4.4** The motor current scale factor shall be checked against a clamp meter on the real chair. — P2 · T · *TBD*

## 6.5 Bus health

**6.5.1** The bridge shall count send failures (e.g. the socket buffer is full or the bus is off) and report them to Diagnostics. — P1 · T

**6.5.2** If CAN transmission fails for 100 ms *(param)*, the bridge shall report a **CAN TX failure** to Safety. — P1 · T

**6.5.3** If no frames at all have been received for 1 s *(param)*, the bridge shall report **CAN RX silent**. — P1 · T
