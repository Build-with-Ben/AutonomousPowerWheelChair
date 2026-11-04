# 7. Diagnostics

Reports the health of the system so problems are visible before they become faults.
Package: Diagnostics (package TBD), using `diagnostic_updater` and `diagnostic_aggregator`.
Output: `/diagnostics`, `/diagnostics_agg`.

Diagnostics reports. It doesn't stop the motors; Safety does (5).

## 7.1 Reporting

**7.1.1** Each node in the command path shall publish `diagnostic_msgs/DiagnosticArray` status at 1 Hz *(param)*. The command path is teleop, the mode manager, control, safety and the bridge. — P1 · T

**7.1.2** Each status shall use the levels OK, WARN, ERROR and STALE. — P1 · I

**7.1.3** `diagnostic_aggregator` shall group statuses by layer: Application, Control & Services, Communication, Hardware. — P2 · I

## 7.2 Monitored values

**7.2.1** Battery: level in %, with a warning below 20% and an error below 10% (matching 5.5). — P1 · T

**7.2.2** Motor current: present value and peak since startup. — P1 · T

**7.2.3** CAN bus:

- **7.2.3.1** frames sent and received per second
- **7.2.3.2** send failures
- **7.2.3.3** frames that couldn't be decoded
- **7.2.3.4** joystick send jitter (6.3.7)

— P1 · T

**7.2.4** Power module: time since the last heartbeat. — P1 · T

**7.2.5** Command path: the rate of each `/cmd_vel/*` topic. — P2 · T

**7.2.6** Raspberry Pi: CPU load, CPU temperature, memory use, free disk and under-voltage or throttling flags. — P2 · T

**7.2.7** Network: whether the link to the operator's machine is up, and its latency, when teleop runs on a different machine. — P3 · T

## 7.3 Access

**7.3.1** The operator shall be able to see a one-line health summary from the development PC, e.g. with `ros2 topic echo /diagnostics_agg` or `rqt_robot_monitor`. — P1 · D

**7.3.2** All diagnostics topics shall be included in the default log recording (9.3.1). — P1 · I
