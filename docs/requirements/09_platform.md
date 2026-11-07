# 9. Platform

Starts the system and records what it does. Package: `wheelchair_bringup`.

## 9.1 Initialization

**9.1.1** A single launch file shall start the full stack: socketcan sender and receiver, bridge, safety, control, mode manager, teleop and diagnostics. — P1 · T

**9.1.2** Launch arguments shall select:

- **9.1.2.1** CAN interface (`can0` / `vcan0`)
- **9.1.2.2** `tx_enabled` (default `false`)
- **9.1.2.3** limits profile (`bringup` / `normal`)
- **9.1.2.4** which nodes run on this machine (all, Pi-side only, or operator-side only)

— P1 · T

**9.1.3** The launch file shall start nodes in this order: CAN interface, bridge and safety, then control and mode manager, then input sources. A command source shall not run before safety is up. — P2 · I

**9.1.4** On the Raspberry Pi, a systemd service shall:

- **9.1.4.1** bring up `can0` at 125 kbit/s at boot
- **9.1.4.2** start the Pi-side stack after `can0` is up
- **9.1.4.3** restart a crashed stack after 5 s *(param)*, ending in Idle with the Startup fault latched (5.7.2)

— P2 · T

**9.1.5** `tx_enabled` shall be `false` after every boot unless the operator's config sets it otherwise. — P1 · I

## 9.2 Configuration

**9.2.1** All parameters shall be kept in YAML files in `wheelchair_bringup/config/`. — P1 · I

**9.2.2** Settings that differ between machines (dev PC and Pi) shall be in separate override files. The shared defaults shall stay the same on both machines. — P1 · I

**9.2.3** Both machines shall use the same `ROS_DOMAIN_ID`, set in one documented place. — P1 · I

## 9.3 Logging

**9.3.1** A launch option shall record a rosbag of the following, to `~/wheelchair_logs/` with a timestamped name:

- **9.3.1.1** `/cmd_vel/*`
- **9.3.1.2** `/rnet/status`
- **9.3.1.3** `/safety/state`
- **9.3.1.4** `/mode/state`
- **9.3.1.5** `/diagnostics*`
- **9.3.1.6** `/to_can_bus` and `/from_can_bus`

— P1 · T

**9.3.2** Recording shall be on by default for every test sequence run. — P1 · I

**9.3.3** Node logs shall include faults (5.6.3) and mode changes (3.3.2) at WARN level or higher. — P1 · I

**9.3.4** Logs shall be rotated or pruned to keep free space on the Pi above 1 GB *(param)*. — P2 · T

**9.3.5** Every recorded bag shall include the git commit hash of the code that produced it, so results can be traced to a code version. — P2 · I
