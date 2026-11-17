# Handling the chair's joystick module (requirement 6.2.3)

**Decision:** keep the chair's joystick module (JSM) connected and working, and use the
**FollowJSM** method from open-rnet. The bridge sends its own joystick frame straight after each
centred frame from the JSM. Decided 2026-11-17.

## The problem

The chair's JSM sends the joystick frame `02000200` every 10 ms (measured on our chair: 1,000
frames in 10 s, 10.0 ms period, 1.8 ms jitter). The power module (PM) drives from that frame.
The PM also expects the JSM's heartbeats `00E` (50 ms) and `03C30F0F` (100 ms), and per open-rnet
it needs `00E` before its startup handshake. So we can't just unplug the JSM, and we can't send
`02000200` on our own 10 ms timer: two nodes sending the same ID at the same moment corrupt each
other's frames.

## Options

From open-rnet ([RNET_PROTOCOL_GUIDE.md](https://github.com/redragonx/open-rnet/blob/af175e2/docs/RNET_PROTOCOL_GUIDE.md),
[JoyLocal.py](https://github.com/redragonx/open-rnet/blob/af175e2/contrib/can2RNET/JoyLocal.py)):

| Method | How it works | If our software stops | Chair's own joystick |
|---|---|---|---|
| **FollowJSM** | Wait for each JSM joystick frame and send ours within ~1 ms. JoyLocal only sends when the JSM frame is exactly centred (`0000`). | The JSM keeps sending `0000`, so the chair stops. | Works, and overrides ours. |
| JSMerror | Send an error frame (`0C000000#` ×3) so the JSM stops sending joystick frames, then send our own every 10 ms. | No joystick frames, so the PM stops. | Disabled until the chair is power-cycled. |
| EmulateJSM | Unplug the JSM and replay its full startup handshake, including the XOR serial challenge. | Depends on our code. | Not present. |

## Why FollowJSM

1. **The operator always wins.** Moving the chair's own joystick makes its frames non-zero, and
   the bridge stops sending. That's a manual override that doesn't depend on any of our nodes, and
   it's stronger than the gamepad deadman (2.4).
2. **It fails safe on the bus.** If the bridge, Safety or the whole computer stops, the JSM's
   centred frames keep coming and the PM stops the chair (1.4.1, 5.7.1, 8.3.3.4).
3. **Nothing is disabled or spoofed.** No error injection, no power cycle to recover, no serial
   authentication.
4. **It's the method open-rnet calls safest**, and it's the default in their main script.

JSMerror is kept as a fallback if FollowJSM turns out to be unreliable on this chair. EmulateJSM
isn't pursued.

## Consequences for the design

| Area | Change |
|---|---|
| Send timing (6.3.1, 6.3.3) | The bridge replies to each JSM frame instead of running its own 10 ms timer. With no JSM frame, it sends nothing. |
| Socket (6.1.3, 1.3.3) | A 1 ms reply can't go through ROS topics and `ros2_socketcan` (several DDS hops). `rnet_bridge` opens the SocketCAN socket itself, and mirrors what it receives and sends on `/from_can_bus` and `/to_can_bus` for logging. |
| Override (6.3.8) | Send only when the JSM frame is centred. After any deflection, wait for the JSM to be centred for 0.5 s *(param)* before sending again, so a stick passing through centre doesn't hand control back mid-movement. |
| Stale input (6.3.4) | Stop replying. The JSM's centred frame then applies. |
| Timing check (6.3.7) | Measure the delay from each JSM frame to our reply, and count replies later than 1 ms. |
| Chair speed (6.3.9) | Optionally set the chair's own maximum speed to its lowest setting (`0A040200#00`) before sending, as a limit inside the PM. Still to be checked on the bench. |

## Risks

| Risk | Mitigation |
|---|---|
| A reply misses the 1 ms window. | The PM uses the JSM's `0000` for that cycle: a stutter, not a lurch. Count misses (6.3.7) and report them to Diagnostics. |
| The 1 ms window is tight. At 125 kbit/s our 2-byte extended frame takes ~0.7–0.8 ms on the wire, and higher-priority frames (for example `00E`) can delay it. | Raw socket in the bridge, with the reply prebuilt and sent from the receive loop. Measure on `vcan0`, then on the bench. If too slow on the Pi, move the reply loop to a small C helper. |
| Our own frames echoed back look like JSM frames. | Raw sockets don't receive their own frames by default (`CAN_RAW_RECV_OWN_MSGS` off). Keep it off, and test it. |
| It's unclear exactly what "within 1 ms" is measured from, and how the PM treats late frames. | Measure on the bench, wheels off the ground, before relying on it (8.3.3). |

## Verification

1. **`vcan0`:** a test fixture plays a JSM, sending `02000200#0000` every 10 ms. Check our reply
   comes after each one, within 1 ms, and that nothing is sent when the fixture sends a deflected
   frame (8.3.2).
2. **Bench, no motion:** send a tone frame to prove the adapter can transmit on R-Net (8.3.4).
3. **Bench, wheels off the ground:** run the forward/back test. Confirm that moving the chair's
   joystick takes over, and run the 8.3.3 stop tests. Record reply delays from `candump -td`.

## Licence note

open-rnet is GPLv3. This project implements the method from the published protocol description,
not by copying their code.
