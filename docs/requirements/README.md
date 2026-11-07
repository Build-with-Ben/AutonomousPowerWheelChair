# Requirements

Requirements for the R-Net wheelchair base, grouped by domain. The first goal is a simple,
safe **input → control → output** loop with safety and diagnostics in between. Object detection
and avoidance are the long-term goal and are captured in [11_future_autonomy.md](11_future_autonomy.md).

See [../layer-stack.md](../layer-stack.md) for the layers and [../data-flow.md](../data-flow.md)
for how commands move between modules. See [../test_plan.md](../test_plan.md) for how each
requirement is verified.

## Domains

| # | Domain | File | Layer | Package(s) |
|---|---|---|---|---|
| 1 | System | [01_system.md](01_system.md) | All | All |
| 2 | Operator Input | [02_operator_input.md](02_operator_input.md) | Application | `wheelchair_teleop` |
| 3 | Mode Management | [03_mode_management.md](03_mode_management.md) | Control & Services | Mode Manager (package TBD) |
| 4 | Motor Control | [04_motor_control.md](04_motor_control.md) | Control & Services | `wheelchair_control` |
| 5 | Safety | [05_safety.md](05_safety.md) | Control & Services | `wheelchair_safety` |
| 6 | CAN Communication | [06_can_communication.md](06_can_communication.md) | Communication | `rnet_bridge`, `ros2_socketcan` |
| 7 | Diagnostics | [07_diagnostics.md](07_diagnostics.md) | Control & Services | Diagnostics (package TBD) |
| 8 | Test Sequences | [08_test_sequences.md](08_test_sequences.md) | Application | `wheelchair_test_sequences` |
| 9 | Platform | [09_platform.md](09_platform.md) | Platform | `wheelchair_bringup` |
| 10 | Hardware | [10_hardware.md](10_hardware.md) | Embedded | — |
| 11 | Future Autonomy | [11_future_autonomy.md](11_future_autonomy.md) | Application, Services | `wheelchair_autonomy`, `wheelchair_odometry` |

## Conventions

**Numbering.** Each requirement has an ID in the form `D.S.R`:

- `D` is the domain number from the table above.
- `S` is the section within that domain.
- `R` is the requirement within that section.
- A fourth level, `D.S.R.n`, is a sub-requirement that refines its parent.

IDs never get reused. If a requirement is removed, mark it **Deleted** and leave its number in place.

**Wording.**

- **shall**: mandatory.
- **should**: a goal. Missing it is allowed, but the reason must be recorded.
- **may**: optional.

**Verification method.** Every requirement ends with how it will be checked:

| Code | Method | Meaning |
|---|---|---|
| T | Test | Run it and measure the result (e.g. `candump`, `ros2 topic hz`, a unit test). |
| D | Demonstration | Operate it and observe the behavior. No measurement. |
| I | Inspection | Read the code, config or wiring. |
| A | Analysis | Calculation or reasoning (e.g. a stopping-distance estimate). |

**Parameters.** Numbers marked *(param)* must be ROS parameters, not hard-coded values. The number
given is the starting default. **TBD** marks a value still to be measured on the real chair.

**Priority.**

- **P1**: needed for the first powered drive.
- **P2**: needed before driving with the wheels on the ground at normal speed.
- **P3**: later.
