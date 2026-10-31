# wheelchair_test_sequences

Scripted motion profiles for bring-up and calibration (e.g. creep forward,
straight line, spin in place, step response). Run with wheels off the ground first.

- Publishes: `/cmd_vel/test` (geometry_msgs/Twist)
- Sequences abort immediately if the safety layer reports a stop.
