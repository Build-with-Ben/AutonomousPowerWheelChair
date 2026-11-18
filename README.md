# AutonomousPowerWheelChair
Autonomous Power Wheel Chair (Quickie 6) using Open R-Net

## Licence

Apache-2.0 — see [LICENSE](LICENSE).

The R-Net protocol details used here come from the
[open-rnet](https://github.com/redragonx/open-rnet) project's published documentation, and
the device nibbles and joystick signs were measured on this chair (see
[docs/analysis/jsm_handling.md](docs/analysis/jsm_handling.md)). open-rnet is GPLv3; no
code from it is copied into this repository.

## Safety

This drives a powered wheelchair, a real machine that can injure someone. The bridge does
not transmit unless `tx_enabled` is true, Safety starts with a latched fault that needs an
operator reset, and the bring-up speed limits are 20% of normal. Keep the drive wheels off
the ground and a hardware stop within reach when testing. The software is provided without
warranty of any kind, as stated in the licence.
