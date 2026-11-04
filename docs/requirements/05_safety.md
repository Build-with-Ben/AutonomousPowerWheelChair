# 5. Safety

The last software gate before the motors. Package: `wheelchair_safety`.
Input: `/cmd_vel/mux`, `/rnet/status`, `/safety/estop`. Output: `/cmd_vel/safe`, `/safety/state`.

Software safety supplements the hardware stop (10.3) and never replaces it.

## 5.1 Pass-through

**5.1.1** Safety shall copy `/cmd_vel/mux` to `/cmd_vel/safe` only while no stop condition in 5.2–5.4 is active. — P1 · T

**5.1.2** Whenever any check fails, Safety shall publish zero velocity on `/cmd_vel/safe`. — P1 · T

**5.1.3** Safety shall publish `/cmd_vel/safe` at a fixed 50 Hz *(param)* whatever rate its input arrives at, so that the bridge always has a fresh command. — P1 · T

## 5.2 Command watchdog

**5.2.1** If `/cmd_vel/mux` hasn't been received for 200 ms *(param)*, Safety shall output zero. — P1 · T

**5.2.2** A stale command alone shall not latch a fault. Pass-through shall resume when fresh commands return. — P1 · T

## 5.3 Power module monitoring

**5.3.1** If no power module heartbeat (frame `0C140X00`, sent about once per second) has been received for 2.5 s *(param)*, Safety shall output zero and latch a **PM heartbeat lost** fault. — P1 · T

**5.3.2** If motor current (frame `14300X00`) stays above the limit for 2 s *(param)*, Safety shall output zero and latch an **Over-current** fault. The limit is *TBD* (param) and shall be measured on the real chair. — P2 · T

**5.3.3** If the power module reports an error or trip state, Safety shall output zero and latch a **PM fault**. The frames that signal this are *TBD*, from the open-rnet frame dictionary and bus captures. — P2 · T

**5.3.4** If `/rnet/status` itself hasn't arrived for 1 s *(param)*, Safety shall latch a **Bridge status lost** fault. — P1 · T

**5.3.5** If the bridge reports a **CAN TX failure** (6.5.2), Safety shall output zero and latch that fault. — P1 · T

## 5.4 E-stop

**5.4.1** If `/safety/estop` is `true`, Safety shall output zero and latch an **E-stop** fault within one output cycle (20 ms). — P1 · T

**5.4.2** Safety shall read the state of the hardware e-stop (10.3.4) and treat an open e-stop circuit like 5.4.1. — P2 · T

**5.4.3** The e-stop shall stay latched until `/safety/estop` is `false` **and** a reset is requested (5.6.2). — P1 · T

## 5.5 Low battery

**5.5.1** When battery level (frame `1C0C0X00`) falls below 20% *(param)*, Safety shall raise a **Low battery** warning without stopping. — P2 · T

**5.5.2** When battery level falls below 10% *(param)*, Safety shall reduce the speed limit to 25% *(param)* of normal. — P2 · T

## 5.6 Fault state and reset

**5.6.1** Safety shall publish `wheelchair_msgs/SafetyState` on `/safety/state` at 10 Hz *(param)*, and immediately on every change. The message shall contain:

- **5.6.1.1** `ok`: `true` only when no fault is latched and no e-stop is active.
- **5.6.1.2** `faults`: the list of every active fault.
- **5.6.1.3** `latched`: `true` while any fault is latched.

— P1 · T

**5.6.2** A `/safety/reset` service (`std_srvs/Trigger`) shall clear latched faults whose causes are no longer present. It shall refuse, and give the reason, if any cause is still present. — P1 · T

**5.6.3** Every fault being raised or cleared shall be logged with its time, the fault name and the values involved. — P1 · I

## 5.7 Safety node failure

**5.7.1** If the Safety node itself stops, the bridge's own stale-input check (6.3.4) shall stop the motors. — P1 · T

**5.7.2** Safety shall start with a latched **Startup** fault, so that an operator reset is needed before the first drive. — P1 · T
