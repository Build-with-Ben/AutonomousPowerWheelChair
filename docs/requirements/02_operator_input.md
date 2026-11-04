# 2. Operator Input

Manual control from a handheld gamepad. Package: `wheelchair_teleop`. Output: `/cmd_vel/teleop`.

## 2.1 Input device

**2.1.1** The system shall accept manual input from a USB or Bluetooth gamepad through the standard `joy` package (`/joy`). — P1 · D

**2.1.2** The gamepad button and axis mapping shall be set in a config file, so a different controller can be used without code changes. — P1 · I

**2.1.3** If the gamepad disconnects, the system shall detect it within 0.5 s *(param)*. — P1 · T

## 2.2 Command generation

**2.2.1** Teleop shall publish `geometry_msgs/Twist` on `/cmd_vel/teleop`. — P1 · T

- **2.2.1.1** One stick axis shall map to `linear.x` (forward/back).
- **2.2.1.2** One stick axis shall map to `angular.z` (turn).
- **2.2.1.3** All other Twist fields shall be zero.

**2.2.2** Teleop shall publish at a fixed rate of 20 Hz *(param)* while the deadman is held, including when the sticks are centered. — P1 · T

**2.2.3** Stick input within a deadzone of ±0.1 *(param)* of center shall be treated as zero. — P1 · T

**2.2.4** Full stick deflection shall map to the maximum teleop speeds: 0.5 m/s *(param)* linear and 1.0 rad/s *(param)* angular. Motor Control (4.2) applies the final limits on top of these. — P1 · T

**2.2.5** Teleop should provide a speed-scale control (e.g. shoulder buttons) with at least three steps, starting at the lowest step on startup. — P2 · D

## 2.3 Deadman and stop

**2.3.1** Teleop shall publish non-zero commands only while a deadman button is held. — P1 · T

**2.3.2** When the deadman is released, teleop shall publish one zero command and then stop publishing. — P1 · T

**2.3.3** When the gamepad disconnects (2.1.3), teleop shall stop publishing. The stale-command watchdog (5.2) then stops the motors. — P1 · T

**2.3.4** A dedicated gamepad button shall publish `true` on `/safety/estop`. This is a software e-stop only and does not replace the hardware stop (10.3). — P1 · T

**2.3.5** A dedicated button combination shall request the Manual mode from Mode Management (3.2). — P2 · D

## 2.4 Priority

**2.4.1** Operator input shall have the highest priority of all command sources (see 4.1). — P1 · T

**2.4.2** While the operator holds the deadman, any running test sequence or autonomy shall be overridden. — P1 · T
