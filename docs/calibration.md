# Open Calibrations and Bench Measurements

Values that can only be measured on the real chair, and the limits that stay disabled
until they are. Each entry names the requirement it satisfies so the trace stays honest.

Nothing here blocks driving on blocks. Everything here blocks trusting a *number*.

## 1. Motor current scale — blocks the over-current fault

**Requirements:** 6.4.4 (scale), 5.3.2 (limit) · **Status:** not measured · **Priority:** highest

`current_scale` is `1.0`, meaning `/rnet/status.motor_current_a` reports **raw counts from
frame `14300X00`, not amps**. Because the scale is unknown, `current_limit` is `-1.0`,
which disables the over-current check entirely. Right now nothing stops a stalled motor.

A 2026-11-24 bench run with the wheels off the ground reported `motor_current_a` peaking at
`9.0`. That is 9 raw counts and must not be used to set a limit.

To measure:

1. Clamp meter on one motor phase, wheels off the ground.
2. Drive at a few steady speeds; record the clamp reading against `motor_current_a`.
3. Repeat with a loaded wheel (hand brake, or on the ground against a kerb) to get a high
   point — the interesting part of the curve is near stall, not near idle.
4. `current_scale` = measured amps / raw counts. Set it in `bringup.yaml`.
5. Only then choose `current_limit`, comfortably above the worst *normal* driving current
   and below the motor's continuous rating.

Until step 4, treat every current reading as unitless.

## 2. Joystick percent to actual speed — blocks meaningful speed limits

**Requirement:** 4.4.2 · **Status:** not measured · **Priority:** high

The bridge converts a velocity command to a joystick percent with
`y = linear_x / linear_limit * max_pct`, where `linear_limit` (0.5 m/s) is *assumed* to be
the speed the chair reaches at full deflection. Nobody has checked that. Until someone
does, every `m/s` figure in this stack is a scaling convention rather than a speed.

Note the two caps **multiply**, which is easy to miss: `wheelchair_control`'s `bringup`
profile allows 20% of 0.5 m/s, and the bridge then maps that against a 0.5 m/s full scale
capped at `max_pct` 20. The chair therefore sees **4%**, not 20%.

| control profile | command cap | `max_pct` | joystick sent |
|---|---|---|---|
| `bringup` | 0.10 m/s | 20 | **4%** |
| `bringup` | 0.10 m/s | 40 | 8% |
| `normal` | 0.50 m/s | 20 | 20% |
| `normal` | 0.50 m/s | 40 | 40% |

To measure: mark a known distance, drive it at a few `max_pct` values with a stopwatch, and
record percent against m/s. Then set `linear_limit` to the real full-deflection speed so
the `m/s` numbers mean something.

## 3. Chair-side speed limit frame — an unused safety belt

**Requirement:** 6.3.9 · **Status:** not tried · **Priority:** medium

The bridge should be able to set the chair's own maximum speed to its lowest setting with
`0A040X00#00` before it starts sending, giving a limit inside the power module that no
software bug can exceed. Its effect on this chair has never been checked. Worth a bench
test, because it is the only limit that does not depend on our own code being correct.

## 4. Power-module fault frames

**Requirement:** 5.3.3 · **Status:** not identified · **Priority:** medium

Which frames signal a PM error or trip is still unknown, so Safety cannot latch a PM fault.
Needs bus captures during a deliberate fault (stall, over-temperature) cross-referenced
against the open-rnet frame dictionary.

## 5. Hardware stop method

**Requirement:** 10.3.5 · **Status:** undecided · **Priority:** high before wheels-on-ground

Whether the hardware stop cuts power-module power or issues an R-Net-level stop is still
open, and so is the power module's braking behaviour in each case. This should be settled
before anyone rides the chair.
