# Test Plan

How every requirement in [requirements/](requirements/README.md) gets verified, and how those
checks run automatically on each pull request.

## 1. Framework

| Need | Tool | Why |
|---|---|---|
| Unit tests | **pytest** | The default for `ament_python` packages; `colcon test` already runs it. |
| Node / topic tests | **launch_testing** (pytest-based) | Starts real ROS nodes from a launch description and asserts on topics. Ships with ROS 2 Jazzy. |
| Style and lint | **ament_flake8**, **ament_pep257** | Already generated in every package's `test/` folder. |
| Coverage | **pytest-cov**, via `colcon test --pytest-with-coverage` | Line coverage per package, no extra config. |
| CAN bus tests | **vcan** kernel module + **python-can** | A virtual CAN bus behaves like `can0`, so frames and timing can be checked without hardware. |
| CI | **GitHub Actions** with the `ros:jazzy-ros-base` container | Same OS and ROS version as the dev PC and the Pi. Free for public repos. |

We use one framework end to end: pytest. Every level below L4 is a pytest test, and `colcon test`
runs it locally exactly the way CI does.

## 2. Test levels

| Level | What | Tools | Hardware | Runs where | Runs when |
|---|---|---|---|---|---|
| **L0 Static** | Lint, docstrings, build | flake8, pep257, `colcon build` | None | CI | Every push and PR |
| **L1 Unit** | Pure logic: frame encode/decode, scaling, limits, state machines, watchdog timing | pytest | None | CI + dev PC | Every push and PR |
| **L2 Node** | One or more nodes talking over topics: e.g. safety zeroes `/cmd_vel/safe` when input stops | launch_testing | None | CI + dev PC | Every push and PR |
| **L3 Bus** | Full stack down to SocketCAN on `vcan0`: frame IDs, 10 ms period, jitter, bus-loss behavior | launch_testing + python-can | None (vcan) | CI host job + dev PC | Every PR (once the bridge has TX code) |
| **L4 Bench (HIL)** | Real transceiver and real power module, **wheels off the ground** | Checklist + rosbag | Dev PC or Pi + chair | Manual | Before every release tag |
| **L5 Field** | Wheels on the ground, open area | Checklist + rosbag | Pi + chair | Manual | Before raising speed limits or adding features |

L0–L3 are automated and gate merges. L4–L5 are manual, because they move real motors. Results go
in the PR or release notes, with a rosbag attached.

## 3. Writing testable code

These rules keep most tests at L1, where they're fast and need no ROS runtime.

**3.1 Separate logic from the node.** Each package keeps its logic in plain Python modules with no
`rclpy` import. The `Node` class only wires topics and timers to those functions.

```
rnet_bridge/
  rnet_bridge/
    frames.py          # encode_joystick(x, y) -> bytes, decode_status(frame) -> dict   ← L1
    bridge_node.py     # rclpy Node: subscriptions, 10 ms timer, calls frames.py        ← L2/L3
  test/
    test_frames.py     # L1
    test_bridge_node.py  # L2 (launch_testing)
```

**3.2 Inject time.** Watchdogs and timeouts take the current time as an argument (or a clock object),
so a test can step time forward instead of sleeping.

**3.3 No hardware below L3.** L1 and L2 never open a CAN socket. L3 only uses `vcan0`.

**3.4 Parameters, not constants.** Anything marked *(param)* in the requirements is a ROS parameter.
That lets tests set short timeouts and makes it easy to cover edge values.

## 4. Requirement traceability

**4.1 Tag tests with requirement IDs.** Each test names the requirement(s) it verifies:

```python
import pytest

@pytest.mark.req('6.3.2.3')
def test_joystick_values_clamp_instead_of_wrap():
    assert encode_joystick(x=150, y=-150) == bytes([100, 0x9C])  # 0x9C = -100
```

The `req` marker is registered in each package's `setup.cfg`.

**4.2 Verification method decides the level.**

| Method in requirements | Verified by |
|---|---|
| **T** Test | An automated test at L1–L3 wherever possible. Otherwise an L4 checklist step with measured values. |
| **D** Demonstration | An L4 or L5 checklist step. |
| **I** Inspection | The PR review checklist (§6.4). |
| **A** Analysis | A short write-up in `docs/` linked from the requirement. |

**4.3 Trace report.** `tools/req_trace.py` reads the requirements and the `@pytest.mark.req` tags,
then prints which requirements have automated tests. CI adds the report to every run's summary.
For now it only reports. Once the P1 code exists, CI will fail if any P1 **T** requirement has no test.

## 5. Test coverage by domain

| Domain | L1 Unit | L2 Node | L3 Bus | L4 Bench |
|---|---|---|---|---|
| 2 Operator Input | Axis mapping, deadzone, scaling, deadman logic | `/joy` in → `/cmd_vel/teleop` out; release → zero then silence | — | Real gamepad; disconnect stops chair |
| 3 Mode Management | Every allowed and refused transition; fault forces Fault | `/mode/request` service; fault on `/safety/state` → Fault in 50 ms | — | Mode changes logged |
| 4 Motor Control | Speed clamp, reverse limit, accel limiter | twist_mux priority and timeout; mode blocks a source | — | Bring-up profile limits observed |
| 5 Safety | Each check alone: watchdog, heartbeat, current, e-stop, battery; latch and reset | Input stops → zero in 200 ms; heartbeat stops → latch; reset refused while cause present | Kill safety node → bridge sends 0/0 | All 8.3.3 stop tests |
| 6 CAN Communication | Joystick encode (scaling, clamping, sign); status decode for each frame ID | `/cmd_vel/safe` → `/to_can_bus` content; stale input → 0/0 | 10 ms ±2 ms period on `vcan0`; extended IDs; TX failure reported | Device nibble, current scale, PM response |
| 7 Diagnostics | Level thresholds (battery, jitter) | `/diagnostics` published at 1 Hz with right levels | Bus counters move on `vcan0` | Pi temperature and voltage during a run |
| 8 Test Sequences | Step timing and ramp profile from parameters | Refuses start when not OK; aborts on fault in 50 ms | Expected Y values per step on `vcan0` (8.3.2) | Wheels-off forward/back run |
| 9 Platform | — | Launch file starts every node; `tx_enabled` defaults false | — | Pi boots into Idle with `can0` up |
| 10 Hardware | — | — | — | Hardware stop, power, wiring checks |

Domain 1 (System) requirements are covered by the domains that refine them. Domain 11 is P3 and gets
tests when it's built.

## 6. CI/CD pipeline

### 6.1 Continuous integration (`.github/workflows/ci.yml`)

Runs on every pull request, on every push to `main`, and on demand.

```
 checkout ─► rosdep install ─► colcon build ─► colcon test (+coverage) ─► test-result ─► req trace ─► upload results
   L0           L0               L0              L0 lint · L1 · L2           gate           report        artifacts
```

| Stage | Fails the run when |
|---|---|
| Build | Any package fails to build. |
| Test | Any lint, unit or node test fails (`colcon test-result` exits non-zero). |
| Trace | Never, for now (report only). Later: a P1 **T** requirement has no test. |

Artifacts kept for each run: `pytest.xml` per package, coverage XML, and the trace report.

**L3 bus job (added with the bridge TX code).** A second job runs directly on the Ubuntu runner,
not in a container, because loading `vcan` needs the host kernel. It installs ROS with
`ros-tooling/setup-ros`, runs `modprobe vcan`, creates `vcan0` and runs tests marked
`@pytest.mark.vcan`. Those tests are skipped automatically wherever `vcan0` doesn't exist.

### 6.2 Merge gate

Once the workflow has run once, add it to the `protect-main` ruleset under
**Require status checks to pass** → `build-test`. A PR then can't merge with a red build.

### 6.3 Continuous delivery to the Pi

The chair is a moving machine, so delivery is automated up to the Pi but never starts the motors
on its own.

1. **Release tag.** When `main` is ready, and L4 has passed on the bench, tag it: `vX.Y.Z`.
2. **Deploy.** On the Pi, a script (`tools/deploy_pi.sh`, to be added) checks out the tag, runs `rosdep install` and `colcon build`, then restarts the systemd service.
3. **Safe start.** After every deploy the stack boots into Idle with `tx_enabled=false` and the Startup fault latched (5.7.2, 9.1.5). Someone has to be present to enable driving.
4. **Record.** Release notes list the tag, the L4/L5 checklist results and the rosbag names. Each bag records the git commit (9.3.5).

A self-hosted GitHub runner on the Pi could later run the L3 tests on real ARM hardware. It should
never enable CAN transmission to the chair.

### 6.4 PR review checklist (Inspection requirements)

Add to the PR description template:

- [ ] New logic is in a module with no `rclpy` import, with L1 tests.
- [ ] New tests carry `@pytest.mark.req(...)` for the requirements they cover.
- [ ] New timeouts, limits and rates are ROS parameters.
- [ ] Nothing other than `rnet_bridge` publishes to `/to_can_bus` (1.3.3).
- [ ] `tx_enabled` still defaults to `false` (6.3.5).

## 7. Quality targets

| Metric | Target |
|---|---|
| L1 line coverage of logic modules (`frames.py`, limiters, state machines) | ≥ 90% |
| Overall line coverage per package | ≥ 70% |
| P1 **T** requirements with an automated test | 100% before the first wheels-on-ground drive |
| CI run time | Under 10 minutes |
| Flaky tests | Zero. A flaky test is fixed or quarantined the same week. |

## 8. Running tests locally

```bash
cd ~/ros2_ws
colcon build
colcon test --pytest-with-coverage --event-handlers console_direct+
colcon test-result --verbose

# one package, one test file
colcon test --packages-select rnet_bridge --pytest-args -k test_frames

# requirement trace
python3 src/tools/req_trace.py
```

For L3 on the dev PC:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up
colcon test --packages-select rnet_bridge --pytest-args -m vcan
```

## 9. Order of work

1. **Now:** CI with L0–L2, lint passing, the `req` marker and the trace report. *(this branch)*
2. `rnet_bridge/frames.py` and its L1 tests, written test-first from domain 6.
3. Safety logic and its L1 tests, then the L2 node tests for 5.2 and 5.3.
4. Bridge TX timer, plus the L3 `vcan` CI job.
5. Motor Control, Operator Input and Mode Management, each with L1 and L2 tests.
6. First L4 bench run, then the first release tag.
