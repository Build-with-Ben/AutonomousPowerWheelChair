# Decoded R-Net power module status, published on /rnet/status (requirement 6.4).
# See docs/requirements/06_can_communication.md. Frame IDs use the power module device
# nibble (pm_nibble); on the test chair that is 0.

std_msgs/Header header                        # header.stamp = when this status was published

# Power module heartbeat (frame 0C140X00, ~1 s on the test chair)
bool pm_alive                                 # a heartbeat arrived within the timeout
builtin_interfaces/Time pm_last_seen          # time the last heartbeat was received

# Battery level (frame 1C0C0X00, ~1 s)
uint8 battery_pct                             # 0-100
builtin_interfaces/Time battery_last_updated

# Motor current (frame 14300X00, ~250 ms on the test chair)
float32 motor_current_a                       # scaled to amps; scale is TBD (6.4.4)
builtin_interfaces/Time motor_current_last_updated

# Motion state (frames 0C000005 / 0C000006, on event)
bool motors_stopped
bool motors_decelerating

# Bus health (6.4.3, 6.5.3)
uint32 rx_frame_count                         # total frames received since start
uint32 unknown_frame_count                    # frames received that could not be decoded
bool rx_silent                                # no frames received for the silent timeout
