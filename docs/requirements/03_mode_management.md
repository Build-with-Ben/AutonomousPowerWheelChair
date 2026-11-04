# 3. Mode Management

Owns the overall system state and decides which command sources are allowed to drive.
Package: Mode Manager (package TBD). Output: `/mode/state`.

## 3.1 Modes

**3.1.1** The system shall have these modes:

| Mode | Meaning | Allowed sources |
|---|---|---|
| **Idle** | Powered and healthy, not driving | none |
| **Manual** | Operator driving | teleop |
| **Test** | Running a test sequence | test sequences, teleop (override) |
| **Auto** | Autonomous behavior | autonomy, teleop (override) |
| **Fault** | A latched safety fault is active | none |
| **E-stop** | An e-stop is active | none |

— P1 · I

**3.1.2** The current mode shall be published at 1 Hz *(param)* on `/mode/state`, and also immediately on every change. — P1 · T

**3.1.3** The system shall start in **Idle**. — P1 · T

## 3.2 Transitions

**3.2.1** Changing from Idle to Manual, Test or Auto shall require an explicit request, made through a `/mode/request` service. — P1 · T

**3.2.2** Mode Management shall reject a request to leave Idle while Safety reports a fault or an e-stop (5.6). — P1 · T

**3.2.3** Any safety fault shall force **Fault** mode. Any e-stop shall force **E-stop** mode. Both shall take effect within 50 ms *(param)* of the fault or e-stop appearing in `/safety/state`. — P1 · T

**3.2.4** Leaving Fault or E-stop shall require:

- **3.2.4.1** the cause to be cleared, and
- **3.2.4.2** an operator reset (`/safety/reset`).

The system shall then return to **Idle**, never directly to a driving mode. — P1 · T

**3.2.5** When a test sequence finishes or aborts, the system shall return to **Idle**. — P1 · T

**3.2.6** Auto mode shall be unavailable (requests rejected) until requirement 1.1.3 is lifted. — P1 · T

## 3.3 Enforcement

**3.3.1** Motor Control shall pass through only the command sources allowed in the current mode (3.1.1). — P1 · T

**3.3.2** Every mode change shall be logged with its time, the old mode, the new mode and the reason. — P1 · I
