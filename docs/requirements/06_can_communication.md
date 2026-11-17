# 6. CAN Communication

Translates between ROS and the R-Net CAN bus. Package: `rnet_bridge`; Linux SocketCAN.
Input: `/cmd_vel/safe`. Output: `/rnet/status`, plus `/from_can_bus` and `/to_can_bus` as logging mirrors.

The chair's own joystick module (JSM) stays connected. The bridge uses the FollowJSM method: it
replies to each centred JSM joystick frame with its own. See
[../analysis/jsm_handling.md](../analysis/jsm_handling.md).

Frame IDs come from the open-rnet frame dictionary
(https://github.com/redragonx/open-rnet/blob/main/reference/RNET_FRAME_DICTIONARY.md).
In each ID, `X` is the device nibble of the module that sends the frame. Modules have different
numbers: on the test chair the joystick module (JSM) is 2 and the power module (PM) is 0.

## 6.1 Bus interface

**6.1.1** The system shall connect to the R-Net bus as a SocketCAN interface at 125 kbit/s. — P1 · T

**6.1.2** The interface name shall be a launch argument, defaulting to `can0`, so that `vcan0` can be used for testing without hardware. — P1 · I

**6.1.3** `rnet_bridge` shall open the SocketCAN socket itself and send and receive frames on it directly, because the FollowJSM reply (6.3.1) is too time-critical to go through ROS topics. It shall mirror every frame it receives on `/from_can_bus` and every frame it sends on `/to_can_bus` (`can_msgs/Frame`), for logging. — P1 · T

**6.1.4** The system shall support 29-bit extended frame IDs. — P1 · T

## 6.2 Device identification

**6.2.1** The device nibbles shall be parameters, with one for the joystick frames (`joystick_nibble`) and one for the power module status frames (`pm_nibble`), because the two modules have different numbers. — P1 · I

**6.2.2** The correct nibble values for this chair shall be found from a bus capture and recorded in the bringup config. Found on 2026-11-17 (listen-only `candump`): `joystick_nibble` = 2 (`02000200`), `pm_nibble` = 0 (`0C140000`, `1C0C0000`, `14300000`). They still need to go in the bringup config once it exists. — P1 · T

**6.2.3** How the chair's own joystick module is handled shall be decided and documented before the first powered drive. Decided 2026-11-17: the JSM stays connected and the bridge uses FollowJSM ([../analysis/jsm_handling.md](../analysis/jsm_handling.md)). — P1 · A

## 6.3 Command transmission

**6.3.1** While transmission is enabled, `rnet_bridge` shall send one joystick frame `02000X00` (extended ID, DLC 2, X = `joystick_nibble`) after each joystick frame it receives from the JSM, within 1 ms of it. It shall not send joystick frames on its own timer, so if no JSM frame arrives it sends nothing. — P1 · T

**6.3.2** Joystick frame data shall be two signed 8-bit values in the range −100 to +100:

- **6.3.2.1** Byte 0 (X): turn, from `angular.z` scaled by the angular limit. Positive X turns right on R-Net (stick left measured X = −96), so X = −`angular.z` scaled, keeping positive `angular.z` a left turn (1.3.5.3).
- **6.3.2.2** Byte 1 (Y): forward/back, from `linear.x` scaled by the linear limit. Positive Y is forward (stick forward measured Y = +96).
- **6.3.2.3** Values outside the range shall be clamped, never wrapped.

— P1 · T

**6.3.3** The bridge shall always send the most recent `/cmd_vel/safe` value. When it sends shall be set by the JSM frames, not by when commands arrive. — P1 · T

**6.3.4** If no `/cmd_vel/safe` message has arrived for 100 ms *(param)*, the bridge shall stop sending joystick frames, so the JSM's centred frame applies. — P1 · T

**6.3.5** A `tx_enabled` parameter, `false` by default, shall disable all transmission. With it disabled, the bridge decodes and logs only. — P1 · T

**6.3.6** The maximum joystick percent shall be a parameter, `max_pct`, defaulting to 20. — P1 · I

**6.3.7** The bridge should measure the delay from each JSM frame to its reply, and count replies later than 1 ms *(param)*, and report both to Diagnostics (7.2.3). — P2 · T

**6.3.8** The chair's own joystick shall always override the bridge (operator override):

- **6.3.8.1** The bridge shall reply only to JSM frames that are centred (X = 0 and Y = 0). It shall send nothing while the chair's joystick is deflected.
- **6.3.8.2** After any deflected JSM frame, the bridge shall send nothing until the JSM has been centred for 0.5 s *(param)*, so a stick passing through centre doesn't hand control back mid-movement.

— P1 · T

**6.3.9** Before it starts sending, the bridge should set the chair's own maximum speed to its lowest setting (`0A040X00#00`, X = `joystick_nibble`), as a limit inside the power module. Its effect on this chair shall be checked on the bench before it's relied on. — P2 · T · *TBD*

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
