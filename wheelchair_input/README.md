# wheelchair_input

Turns any controller into device-independent operator input, so nothing downstream knows
what kind of device is plugged in.

```
js0 ──> joy_node ──> /joy ──> wheelchair_input ──> /operator/input ──> wheelchair_teleop
         (ROS std)   (raw      (device profile)     (OperatorInput:      (deadman, deadzone,
                      indices)                       named functions)     speed scale, limits)
```

- Subscribes: `/joy` (`sensor_msgs/Joy`, from the `joy` package)
- Publishes: `/operator/input` (`wheelchair_msgs/OperatorInput`)

The message names *functions* — `deadman`, `estop`, `linear`, `angular`, `speed_up`,
`speed_down`, `mode_manual` — not axis or button indices. Everything device specific lives
in a YAML profile under [config/](config/), which is the only thing that changes when the
controller changes (requirement 2.1.2).

The node is a pass-through: one `/joy` message in, one `/operator/input` message out. It
never republishes on a timer, because silence is how a disconnected controller reaches
teleop's timeout (2.1.3, 2.3.3).

## Running it

```sh
# controller -> mapping -> teleop, without the chair
ros2 launch wheelchair_input operator.launch.py

# a different device
ros2 launch wheelchair_input operator.launch.py profile:=atari_remote
```

`joy_node` must run with `autorepeat_rate` above teleop's publish rate. Without it, a stick
held perfectly still produces no `/joy` messages and teleop's input timeout stops the chair
mid-drive. The launch files set it; if you start `joy_node` by hand, pass it yourself. Pass
`deadzone:=0.0` too, because `joy_node`'s own default is 0.05 and teleop owns the deadzone.

The controller is opened by **name**, not by index, so it does not matter how many other
input devices are attached or in what order. `joy_node` retries while the named device is
absent and opens it as soon as it appears, so the machine may boot before the controller is
switched on. Use the name SDL reports, which is not always the kernel's:

```sh
ros2 run joy joy_enumerate_devices     # the name to put in device_name
```

## Writing a profile for a new controller

Each logical control picks a *source* by its `type`:

| Control | Types | Settings |
|---|---|---|
| `linear`, `angular` | `axis` | `index`, `invert`, `scale`, `trim` |
| | `buttons` | `plus`, `minus` — two switches as one axis |
| | `none` | always 0 |
| `deadman`, `estop`, `speed_up`, `speed_down`, `mode_manual` | `button` | `index` |
| | `combo` | `indices` — all held together |
| | `axis` | `index`, `threshold`, `invert` — a trigger as a button |
| | `none` | always false |

Every control defaults to `none`, so a control you leave out is simply unavailable. That
matters most for `deadman`: unmapped means the chair never moves, rather than being
permanently enabled. A typo in a profile is rejected at startup with the offending key
named, instead of silently driving the wrong axis.

To find the indices for a new device, run the probe and follow the prompts:

```sh
ros2 run joy joy_node --ros-args -p autorepeat_rate:=25.0 &
python3 tools/joy_probe.py --seconds 60
```

It reports which axis or button each control moves, including its resting value and the
sign of its deflection, so `index`, `invert` and `trim` can be read straight off.

## Shipped profiles

| Profile | Device |
|---|---|
| `gamepad_generic` | Generic twin-stick USB gamepad (DragonRise 0079:0126), used on the bench |
| `atari_remote` | Worked example of a digital joystick and button remote; no hardware yet |

`atari_remote` has no analog axes at all — it builds both drive axes out of four direction
switches with the `buttons` type. It is there to show that a device like that needs a
profile and nothing else: anything that can publish `sensor_msgs/Joy` works, whether that
is a USB pad, a GPIO reader or an ESP32 over wifi.

## Running more than one controller

Start one node per device, each with its own name and profile, all publishing to
`/operator/input`. The `source` field says which device a message came from.

- Mute a device live: `ros2 param set /wheelchair_input enabled false`
- Restrict which devices may drive: set teleop's `sources` parameter to the profile names
  it should accept (`['']`, the default, accepts any).

Two devices publishing at once are not arbitrated: the most recent message wins. In
practice only one deadman is held at a time, but real arbitration belongs here if several
operators ever need to share the chair.
