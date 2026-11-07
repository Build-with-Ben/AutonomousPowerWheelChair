# 1. System

Requirements that apply to the whole stack. Domain-specific requirements refine these.

## 1.1 Purpose and scope

**1.1.1** The system shall drive an R-Net powered wheelchair base from ROS 2 by injecting commands on the R-Net CAN bus through open-rnet. — P1 · D

**1.1.2** The system shall support three command sources: manual operator input, scripted test sequences, and automatic/autonomous control. — P1 · I

**1.1.3** The first release shall provide only manual input and test sequences. Autonomy shall be limited to placeholder interfaces until the safety and diagnostics requirements are verified. — P1 · I

## 1.2 Software platform

**1.2.1** The system shall run on Ubuntu 24.04 with ROS 2 Jazzy, on both the development PC and the Raspberry Pi. — P1 · I

**1.2.2** All packages shall build with `colcon build` from a fresh clone of the repository into a workspace `src/` folder. — P1 · T

**1.2.3** Each package shall declare all of its run-time dependencies in `package.xml`, so that `rosdep install` installs everything it needs. — P1 · T

**1.2.4** The same source code shall run on the development PC and the Raspberry Pi. Differences between machines shall be handled by configuration only (see 9.2). — P1 · I

## 1.3 Architecture

**1.3.1** Modules shall communicate only through ROS 2 topics, services and actions (the Middleware layer). — P1 · I

**1.3.2** Every motion command shall pass through Motor Control (4) and then Safety (5) before reaching CAN Communication (6). — P1 · I

**1.3.3** `rnet_bridge` shall be the only node that publishes to `/to_can_bus`. — P1 · I

**1.3.4** Custom message and service types shared between packages shall be defined in `wheelchair_msgs`. — P1 · I

**1.3.5** The system shall follow ROS conventions:

- **1.3.5.1** SI units: m, m/s, rad/s, A, V (REP-103).
- **1.3.5.2** Body frame: `base_link`, with x forward and z up (REP-103/105).
- **1.3.5.3** Positive `angular.z` turns the chair left (counter-clockwise viewed from above).

— P1 · I

## 1.4 Fail-safe behavior

**1.4.1** Any loss of the command path shall result in zero motion. Examples: a node crashes, a topic goes silent, the network link drops, CAN transmission fails. — P1 · T

**1.4.2** On startup, the system shall command zero motion until an operator explicitly enables driving (see 3.2). — P1 · T

**1.4.3** The system shall include a hardware stop that does not rely on any software (see 10.3). — P1 · I

## 1.5 Quality

**1.5.1** Python code shall pass the `ament_flake8` and `ament_pep257` tests created with each package. — P2 · T

**1.5.2** Pure logic, such as frame encoding and decoding, scaling, limiting and state transitions, shall have unit tests that run without hardware. — P2 · T

**1.5.3** Changes to `main` shall go through a pull request (branch protection is enabled). — P1 · I
