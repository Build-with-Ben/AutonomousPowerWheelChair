# wheelchair_control

Command arbitration: selects one active velocity source (teleop > test sequences > autonomy) with twist_mux and applies speed and acceleration limits, publishing `/cmd_vel/mux`.
