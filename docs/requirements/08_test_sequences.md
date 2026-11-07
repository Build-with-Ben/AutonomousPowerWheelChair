# 8. Test Sequences

Scripted motion for bring-up, verification and calibration. Package: `wheelchair_test_sequences`.
Output: `/cmd_vel/test`. Services: `/test/start`, `/test/stop`.

## 8.1 Execution

**8.1.1** A test sequence shall start only through `/test/start` (`std_srvs/Trigger`), and only while:

- **8.1.1.1** the system is in Test mode (3.1.1), and
- **8.1.1.2** `/safety/state.ok` is `true`.

— P1 · T

**8.1.2** `/test/stop` shall end a running sequence immediately, sending zero velocity. — P1 · T

**8.1.3** If `/safety/state` reports a fault or the mode leaves Test, a running sequence shall abort within 50 ms *(param)*. — P1 · T

**8.1.4** While a sequence runs, it shall publish `/cmd_vel/test` at 20 Hz *(param)*. — P1 · T

**8.1.5** Every speed, ramp time and hold time in a sequence shall be a parameter. — P1 · I

## 8.2 Forward / backward test (first sequence)

**8.2.1** The forward/backward test shall run these steps, with defaults *(param)* shown:

- **8.2.1.1** Ramp to +0.1 m/s over 1 s.
- **8.2.1.2** Hold +0.1 m/s for 3 s.
- **8.2.1.3** Ramp to 0 and hold for 1 s.
- **8.2.1.4** Ramp to −0.1 m/s over 1 s.
- **8.2.1.5** Hold −0.1 m/s for 3 s.
- **8.2.1.6** Ramp to 0 and finish.

— P1 · T

**8.2.2** The sequence shall log the step name each time a step starts. — P1 · I

**8.2.3** At the end, the sequence shall report a summary:

- **8.2.3.1** result: passed, aborted or failed
- **8.2.3.2** abort reason, if aborted
- **8.2.3.3** peak motor current during each step
- **8.2.3.4** joystick frame count sent during the sequence

— P1 · T

## 8.3 Test procedure

**8.3.1** The first runs shall be done with the drive wheels off the ground, with a person standing at the hardware stop. — P1 · D

**8.3.2** Before any powered run, the full sequence shall be run on `vcan0`. `candump -td vcan0` shall show:

- **8.3.2.1** joystick frames every 10 ms
- **8.3.2.2** the expected Y values for each step
- **8.3.2.3** X = 0 throughout

— P1 · T

**8.3.3** Each of these tests shall pass, wheels off the ground, before the chair drives with its wheels on the ground:

- **8.3.3.1** Releasing the deadman mid-sequence (teleop override) stops the chair.
- **8.3.3.2** A software e-stop mid-sequence stops the chair.
- **8.3.3.3** Killing the safety node stops the chair.
- **8.3.3.4** Killing the bridge stops the chair (the power module stops when joystick frames stop).
- **8.3.3.5** Unplugging the CAN adapter stops the chair.
- **8.3.3.6** The hardware stop stops the chair.

— P1 · D

## 8.4 Additional sequences

**8.4.1** A spin-in-place sequence (±`angular.z`) should be provided to check the turn direction (1.3.5.3). — P2 · T

**8.4.2** A step-response sequence should be provided to measure joystick-percent-to-speed calibration (4.4.2). — P2 · T
