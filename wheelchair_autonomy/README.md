# wheelchair_autonomy

Automatic / autonomous control: anything that drives the chair without a human
on the sticks (simple behaviors now, Nav2 / coverage planning later).

- Publishes: `/cmd_vel/auto` (geometry_msgs/Twist)
- Lower priority than teleop in `wheelchair_control`'s mux.
