"""
R-Net frame encoding and decoding (requirements 6.2, 6.3.2, 6.4.1).

Pure logic, no rclpy. Frame IDs come from the open-rnet frame dictionary. In each
device-specific ID the ``X`` nibble (bits 8-11) is the chair's device nibble (6.2.1).
"""

JOYSTICK_BASE = 0x02000000        # 02000X00
PM_HEARTBEAT_BASE = 0x0C140000    # 0C140X00
MOTOR_CURRENT_BASE = 0x14300000   # 14300X00
BATTERY_BASE = 0x1C0C0000         # 1C0C0X00
MOTORS_STOPPED_ID = 0x0C000005
MOTORS_DECELERATING_ID = 0x0C000006

JOYSTICK_DLC = 2
PCT_LIMIT = 100


def device_id(base, nibble):
    """Return ``base`` with the device nibble ``X`` filled in."""
    if not 0 <= nibble <= 0xF:
        raise ValueError(f'device nibble must be 0..15, got {nibble}')
    return base | (nibble << 8)


def joystick_id(nibble):
    """Return the joystick frame ID ``02000X00`` for this device nibble."""
    return device_id(JOYSTICK_BASE, nibble)


def clamp_pct(value, limit=PCT_LIMIT):
    """Round to an int and clamp to ``±limit``. Never wraps (6.3.2.3)."""
    return int(max(-limit, min(limit, round(value))))


def twist_to_joystick(linear_x, angular_z, linear_limit, angular_limit, max_pct=20):
    """
    Scale a velocity command to joystick percent ``(x, y)``.

    A command at the full speed limit maps to ``±max_pct`` (6.3.2.1, 6.3.2.2, 6.3.6).
    Anything beyond the limit is clamped to ``±max_pct``.
    """
    if linear_limit <= 0 or angular_limit <= 0:
        raise ValueError('speed limits must be positive')
    if not 0 <= max_pct <= PCT_LIMIT:
        raise ValueError(f'max_pct must be 0..{PCT_LIMIT}, got {max_pct}')
    x = clamp_pct(angular_z / angular_limit * max_pct, max_pct)
    y = clamp_pct(linear_x / linear_limit * max_pct, max_pct)
    return x, y


def encode_joystick(x, y):
    """Encode joystick percent as two signed 8-bit bytes: X (turn), Y (forward)."""
    return bytes(clamp_pct(v) & 0xFF for v in (x, y))


def decode_joystick(data):
    """Decode joystick frame data back to ``(x, y)`` percent."""
    if len(data) < JOYSTICK_DLC:
        raise ValueError('joystick frame needs 2 data bytes')
    return tuple(b - 256 if b > 127 else b for b in data[:2])


def decode_status(frame_id, data, nibble, current_scale=1.0):
    """
    Decode a status frame into a dict of RnetStatus fields (6.4.1).

    Returns ``None`` for any frame ID it doesn't know or data it can't decode, so the
    caller can count it (6.4.3) instead of raising.
    """
    data = bytes(data)
    if frame_id == device_id(PM_HEARTBEAT_BASE, nibble):
        return {'pm_alive': True}
    if frame_id == device_id(MOTOR_CURRENT_BASE, nibble):
        if len(data) < 2:
            return None
        raw = int.from_bytes(data[:2], 'little')
        return {'motor_current_a': raw * current_scale}
    if frame_id == device_id(BATTERY_BASE, nibble):
        if len(data) < 1:
            return None
        return {'battery_pct': min(data[0], 100)}
    if frame_id == MOTORS_STOPPED_ID:
        return {'motors_stopped': True, 'motors_decelerating': False}
    if frame_id == MOTORS_DECELERATING_ID:
        return {'motors_stopped': False, 'motors_decelerating': True}
    return None
