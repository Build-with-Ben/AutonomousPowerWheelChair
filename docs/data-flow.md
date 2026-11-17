# Wheelchair Command Flow

How a velocity command travels from an input source to the R-Net power module, and how
status from the bus feeds back into safety. Every source passes through the mux and the
safety gate; only `rnet_bridge` writes to the bus. The chair's own joystick module stays on the
bus, and moving it always overrides the bridge (see [analysis/jsm_handling.md](analysis/jsm_handling.md)).

Solid arrows are the command path to the motors. Dashed arrows are status and fault feedback.

```mermaid
flowchart LR
  subgraph IN["Input sources"]
    direction TB
    JOY(["Gamepad<br/>joy node"]) -->|/joy| TELEOP["wheelchair_teleop"]
    TEST["wheelchair_test_sequences<br/>fwd_back_test"]
    AUTO["wheelchair_autonomy"]
  end

  TELEOP -->|"/cmd_vel/teleop · prio 100"| MUX
  TEST -->|"/cmd_vel/test · prio 50"| MUX
  AUTO -->|"/cmd_vel/auto · prio 10"| MUX

  MUX["wheelchair_control<br/>arbitration + limits"] -->|/cmd_vel/mux| SAFE
  ESTOP(["/safety/estop"]) --> SAFE
  SAFE["wheelchair_safety<br/>watchdog · heartbeat · current · e-stop"] -->|/cmd_vel/safe| BRIDGE

  BRIDGE["rnet_bridge<br/>raw socket · replies to each<br/>centred JSM frame (FollowJSM)"] -->|joystick frame| CAN[("can0")]
  JSM[["Chair joystick module<br/>02000X00 every 10 ms"]] --> CAN
  CAN --> PM[["R-Net power module<br/>→ motors"]]

  PM -.-> CAN
  CAN -.->|JSM frames · PM status| BRIDGE
  BRIDGE -.->|"/from_can_bus · /to_can_bus<br/>(logging mirrors)"| LOG(["rosbag"])
  BRIDGE -.->|"/rnet/status<br/>heartbeat · current · battery"| SAFE
  SAFE -.->|"/safety/state<br/>abort on fault"| TEST
```

## Topics and services

| Name | Type | From → To |
|---|---|---|
| `/cmd_vel/test` | `geometry_msgs/Twist` | test sequence → mux |
| `/cmd_vel/mux` | `geometry_msgs/Twist` | mux → safety |
| `/cmd_vel/safe` | `geometry_msgs/Twist` | safety → bridge |
| `/to_can_bus` | `can_msgs/Frame` | bridge → rosbag (mirror of frames sent) |
| `/from_can_bus` | `can_msgs/Frame` | bridge → rosbag (mirror of frames received) |
| `/rnet/status` | `wheelchair_msgs/RnetStatus` | bridge → safety |
| `/safety/state` | `wheelchair_msgs/SafetyState` | safety → test sequence |
| `/safety/estop` | `std_msgs/Bool` | e-stop → safety |
| `/test/start` | `std_srvs/Trigger` (service) | operator → test sequence |
| `/safety/reset` | `std_srvs/Trigger` (service) | operator → safety |

## Faults that stop the motors

| Fault | Detected by | Trigger |
|---|---|---|
| Stale command | safety | no `/cmd_vel/mux` for 200 ms |
| PM heartbeat lost | safety | no `0C140X00` frame for ~2.5 s |
| Over-current | safety | `14300X00` motor current above the set limit |
| E-stop | safety | `/safety/estop` true |
| Bridge input stale | rnet_bridge | no `/cmd_vel/safe` for 100 ms → stops replying; JSM's 0/0 applies |
| Operator override | rnet_bridge | chair's joystick deflected → stops replying until centred for 0.5 s |
| Bridge or computer down | JSM | bridge sends nothing; JSM's centred frames stop the chair |

Safety faults latch until `/safety/reset` is called. The test sequence aborts as soon as
`/safety/state` reports a fault.
