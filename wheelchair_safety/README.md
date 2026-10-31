# wheelchair_safety

Last software gate before the motors: passes `/cmd_vel/mux` to `/cmd_vel/safe` only while commands are fresh, the e-stop is clear and the power module heartbeat is present, otherwise commands zero.
