# wheelchair_teleop

Manual control: gamepad/joystick -> velocity command.

- Subscribes: `/joy` (sensor_msgs/Joy, from the `joy` package)
- Publishes:  `/cmd_vel/teleop` (geometry_msgs/Twist)
- Deadman button must be held for any non-zero output.
- Highest priority input in `wheelchair_control`'s mux (a human can always override).
