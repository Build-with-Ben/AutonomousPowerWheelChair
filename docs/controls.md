# Operator Controls

What each control on the handheld does, and what has to be true before the chair will move.
Written for whoever is driving it.

The mapping lives in a device profile, not in code, so this page describes
[`wheelchair_input/config/xbox_one.yaml`](../wheelchair_input/config/xbox_one.yaml) — the
default. Any other controller works the same way with a different profile; see
[wheelchair_input](../wheelchair_input/README.md) for how to write one.

```
Xbox pad ─▶ dongle ─▶ joy_node ─▶ wheelchair_input ─▶ wheelchair_teleop ─▶ control ─▶ safety ─▶ bridge ─▶ chair
                                   (device profile)     (deadman, deadzone,   (limits)  (gate)
                                                         speed scale)
```

## Xbox One controller

| Control | Function | Behaviour |
|---|---|---|
| **Left stick** | drive | up = forward, left = turn left |
| **LT** (hold) | **deadman** | the chair moves only while this is pulled past halfway |
| **RT** | software e-stop | one pull = one stop request; it **latches** |
| **RB** | speed step up | three steps; see below |
| **LB** | speed step down | |
| **Back + Start** | request Manual mode | needed before any driving |
| **X + Y** | clear a latched e-stop | ignored while LT is held |

Nothing else on the pad is mapped. An unmapped control does nothing rather than doing
something unexpected.

The triggers are analog, so they register as held once pulled past halfway. Both rest at
+1.0 and fall to -1.0 when pulled, which is why the profile inverts them.

## Before it will move

Three independent gates, all of which must be open. This is deliberate: no single mistake
starts the chair.

| Gate | How to open it | Why it exists |
|---|---|---|
| Bridge transmit | launch with `tx_enabled:=true` | the bridge is receive-only by default |
| Safety | `/safety/reset` — a Startup fault is latched at boot | nothing drives until Safety has been told the chair is ready |
| Mode | **Back + Start**, or `/mode/request` | `idle` blocks every command source |
| Deadman | hold **LT** | no non-zero command is published without it |

```sh
ros2 service call /safety/reset std_srvs/srv/Trigger
ros2 service call /mode/request wheelchair_msgs/srv/RequestMode "{mode: manual}"
```

## Stopping

Three ways, fastest first. Know where all three are before you drive.

1. **Release LT.** Teleop publishes one zero command and then goes quiet. This is the
   normal way to stop.
2. **Deflect the chair's own joystick.** The bridge stops replying immediately and the
   chair's own module takes over. This does not depend on our software being correct, so
   it is the most trustworthy stop available.
3. **Pull RT.** The software e-stop. It latches a fault, drops the mode to `estop` and
   commands zero, in about 30 ms end to end. It is a *software* stop and does not replace
   the hardware stop.

Letting go of the controller also stops the chair: with no input for 500 ms teleop falls
silent and the watchdog takes over. The same applies if the controller's battery dies or
it wanders out of range.

## After an e-stop

The stop is sticky on purpose — you should not be able to un-stop the chair by fumbling a
trigger. To recover:

1. **Release LT** completely. Re-arming is refused while the deadman is held.
2. **Press X + Y.** This releases the e-stop and clears the latched fault.
3. **Press Back + Start.** Clearing a fault lands the system in `idle`, never straight
   back into driving, so Manual has to be asked for again.

If X+Y does nothing, check that LT is fully released.

## Speed steps

Teleop starts at the **lowest** step every time it starts. RB steps up, LB steps down.

What the steps actually deliver depends on the launch arguments, because the speed caps
**multiply**: `wheelchair_control`'s profile caps the command, and the bridge then scales
that against full deflection capped at `max_pct`.

Default (`limits_profile:=bringup`, `max_pct:=20`):

| step | forward | reverse | turn |
|---|---|---|---|
| 0, 1, 2 | 4% | 3% | 4% |

All three steps are identical here, because the `bringup` cap of 0.1 m/s is below even the
lowest step's request of 0.125 m/s, so every step clamps to the same value. **The speed
buttons do nothing in the default configuration.**

Bench settings (`limits_profile:=normal`, `max_pct:=40`):

| step | forward | reverse | turn |
|---|---|---|---|
| 0 | 10% | 10% | 10% |
| 1 | 20% | 20% | 20% |
| 2 | 40% | 30% | 40% |

Percentages are of the chair's own joystick range. Reverse is capped at 75% of the forward
limit (requirement 4.2.2), which is an absolute cap rather than a scaling — so it only
takes effect at the top step, where 40% forward becomes 30% reverse.

## Running it

```sh
# default, conservative: 4% to the chair
ros2 launch wheelchair_bringup bringup.launch.py tx_enabled:=true

# bench settings used for the 10/20/40% table above
ros2 launch wheelchair_bringup bringup.launch.py \
    tx_enabled:=true limits_profile:=normal max_pct:=40

# a different controller
ros2 launch wheelchair_bringup bringup.launch.py tx_enabled:=true profile:=gamepad_generic
```

The controller is opened by **name**, so it does not matter which port or index it lands
on, and `joy_node` keeps retrying until it appears — the machine can boot before the
controller is switched on.

## What is not protected yet

**Over-current detection is disabled.** `current_scale` is uncalibrated, so `current_limit`
is `-1.0` and nothing in software stops a stalled or jammed motor. Reported current is in
raw counts, not amps. This matters more at the higher speed settings. See
[calibration.md](calibration.md), item 1.

**Speeds in m/s are a scaling convention, not measured speeds.** Nobody has checked what
the chair actually does at full deflection. See [calibration.md](calibration.md), item 2.

**The hardware stop method is undecided** (requirement 10.3.5). Settle it before anyone
rides the chair.

## Other devices

| Profile | Device | Status |
|---|---|---|
| `xbox_one` | Xbox One pad via the xone dongle | default, verified on the chair |
| `gamepad_generic` | generic twin-stick USB pad | axes measured, **buttons unverified** |
| `atari_remote` | example digital joystick and button remote | no hardware yet |

`gamepad_generic` has not had its buttons confirmed, so probe it with
`tools/joy_probe.py` before driving with it. `rearm` is deliberately left unmapped on both
non-default profiles rather than guessing an index for a control that clears a latched stop.
