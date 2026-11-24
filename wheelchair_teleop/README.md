# wheelchair_teleop

Manual control: device-independent operator input -> velocity command.

- Subscribes: `/operator/input` (`wheelchair_msgs/OperatorInput`, from `wheelchair_input`)
- Publishes: `/cmd_vel/teleop` (`geometry_msgs/Twist`), `/safety/estop` (`std_msgs/Bool`)
- Calls: `/mode/request` (`wheelchair_msgs/RequestMode`) to ask for Manual mode
- Deadman must be held for any non-zero output; releasing it publishes one zero and stops.
- Highest priority input in `wheelchair_control`'s mux (a human can always override).

No axis or button index appears in this package. Which stick drives and which button is the
deadman is decided by a device profile in [`wheelchair_input`](../wheelchair_input), so a
different controller needs a different profile and no change here (requirement 2.1.2).

What stays here is the behaviour that is specified independently of the device: the
deadzone (2.2.3), the speed-scale steps (2.2.5), the maximum speeds (2.2.4), the 20 Hz
publish rate (2.2.2), the deadman rules (2.3.1, 2.3.2) and the input timeout that stops
publishing when the controller goes quiet (2.1.3, 2.3.3).

```sh
# the whole operator chain, without the chair
ros2 launch wheelchair_input operator.launch.py
```

The operator-facing control reference — what each button does, the gates before the
chair moves, and how to recover from an e-stop — is in [docs/controls.md](../docs/controls.md).
