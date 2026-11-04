# 4. Motor Control

Chooses one active command source and shapes the command for the drive motors.
Package: `wheelchair_control`. Input: `/cmd_vel/{teleop,test,auto}`. Output: `/cmd_vel/mux`.

Scope: the two drive motors only. Implements and actuators are out of scope (see 11.5).

## 4.1 Command arbitration

**4.1.1** Motor Control shall select one command source at a time, using `twist_mux` with these priorities *(param)*:

- **4.1.1.1** teleop: 100
- **4.1.1.2** test: 50
- **4.1.1.3** auto: 10

— P1 · T

**4.1.2** A source shall drop out of arbitration if it hasn't published for 0.25 s *(param)*. — P1 · T

**4.1.3** Motor Control shall block every source that the current mode doesn't allow (3.3.1). — P1 · T

**4.1.4** When no source is active, Motor Control shall publish zero velocity on `/cmd_vel/mux` at 20 Hz *(param)*. — P1 · T

## 4.2 Speed limits

**4.2.1** Motor Control shall clamp `linear.x` to ±0.5 m/s *(param)* and `angular.z` to ±1.0 rad/s *(param)*. — P1 · T

**4.2.2** Reverse speed shall be limited to 50% *(param)* of the forward limit. — P1 · T

**4.2.3** Bring-up limits shall be available as a separate parameter set, at 20% of normal limits, and shall be the default until the wheels-on-ground tests in 8.3 pass. — P1 · I

## 4.3 Acceleration limits

**4.3.1** Linear acceleration shall be limited to 0.5 m/s² *(param)*. — P1 · T

**4.3.2** Angular acceleration shall be limited to 1.5 rad/s² *(param)*. — P1 · T

**4.3.3** Deceleration toward zero may use a higher limit, 1.0 m/s² *(param)*, so the chair stops faster than it speeds up. — P2 · T

**4.3.4** Acceleration limits shall not delay a safety stop. Safety (5.3) commands zero immediately, and the power module's own deceleration applies. — P1 · I

## 4.4 Units and mapping

**4.4.1** Every Motor Control input and output shall be in SI units (m/s, rad/s). Converting to R-Net joystick percent is the CAN Communication domain's job (6.3). — P1 · I

**4.4.2** The relationship between joystick percent and actual speed shall be measured on the real chair and recorded as a calibration parameter. — P2 · T · *TBD*
