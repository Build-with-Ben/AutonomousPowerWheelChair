"""L1 tests for R-Net frame encoding and decoding (domain 6)."""

import pytest
from rnet_bridge import frames


# Device numbers measured on the test chair (2026-11-17 bus capture).
JSM_NIBBLE = 0x2
PM_NIBBLE = 0x0


@pytest.mark.req('6.2.1')
@pytest.mark.parametrize('nibble, expected', [
    (0x0, 0x02000000),
    (0x3, 0x02000300),
    (0xF, 0x02000F00),
])
def test_joystick_id_uses_device_nibble(nibble, expected):
    assert frames.joystick_id(nibble) == expected


@pytest.mark.req('6.2.1', '6.2.2')
def test_measured_joystick_id():
    assert frames.joystick_id(JSM_NIBBLE) == 0x02000200


@pytest.mark.req('6.2.1')
@pytest.mark.parametrize('nibble', [-1, 16])
def test_device_nibble_out_of_range_is_rejected(nibble):
    with pytest.raises(ValueError):
        frames.joystick_id(nibble)


@pytest.mark.req('6.1.4')
def test_status_ids_need_29_bits():
    for base in (frames.JOYSTICK_BASE, frames.PM_HEARTBEAT_BASE, frames.MOTOR_CURRENT_BASE,
                 frames.BATTERY_BASE, frames.MOTORS_STOPPED_ID):
        frame_id = frames.device_id(base, 0xF) if base & 0xFF == 0 else base
        assert 0x7FF < frame_id <= 0x1FFFFFFF


# --- 6.3.2 joystick encoding -------------------------------------------------------------

@pytest.mark.req('6.3.2')
def test_joystick_frame_is_two_bytes():
    assert len(frames.encode_joystick(0, 0)) == frames.JOYSTICK_DLC


@pytest.mark.req('6.3.2.1', '6.3.2.2')
@pytest.mark.parametrize('x, y, expected', [
    (0, 0, bytes([0x00, 0x00])),
    (1, -1, bytes([0x01, 0xFF])),
    (100, 100, bytes([0x64, 0x64])),
    (-100, -100, bytes([0x9C, 0x9C])),
    (20, -20, bytes([0x14, 0xEC])),
])
def test_joystick_encodes_signed_8_bit(x, y, expected):
    assert frames.encode_joystick(x, y) == expected


@pytest.mark.req('6.3.2.1', '6.3.2.2')
def test_byte_0_is_turn_and_byte_1_is_forward():
    assert frames.encode_joystick(x=5, y=0) == bytes([5, 0])
    assert frames.encode_joystick(x=0, y=5) == bytes([0, 5])


@pytest.mark.req('6.3.2.3')
def test_joystick_values_clamp_instead_of_wrap():
    assert frames.encode_joystick(x=150, y=-150) == bytes([100, 0x9C])  # 0x9C = -100
    assert frames.encode_joystick(x=128, y=-129) == bytes([100, 0x9C])
    assert frames.encode_joystick(x=1000, y=-1000) == bytes([100, 0x9C])


@pytest.mark.req('6.3.2')
@pytest.mark.parametrize('x, y', [(0, 0), (1, -1), (100, -100), (-37, 64)])
def test_joystick_round_trip(x, y):
    assert frames.decode_joystick(frames.encode_joystick(x, y)) == (x, y)


# --- 6.3.2 / 6.3.6 scaling from SI units ---------------------------------------------------

@pytest.mark.req('6.3.2.1', '6.3.2.2', '6.3.6')
def test_full_limit_maps_to_max_pct():
    assert frames.twist_to_joystick(0.5, -1.0, 0.5, 1.0, max_pct=20) == (20, 20)
    assert frames.twist_to_joystick(-0.5, 1.0, 0.5, 1.0, max_pct=20) == (-20, -20)


@pytest.mark.req('6.3.2.2')
def test_linear_scales_proportionally():
    assert frames.twist_to_joystick(0.25, 0.0, 0.5, 1.0, max_pct=20) == (0, 10)
    assert frames.twist_to_joystick(0.1, 0.0, 0.5, 1.0, max_pct=100) == (0, 20)


@pytest.mark.req('6.3.2.1', '1.3.5.3')
def test_left_turn_sends_negative_x():
    # Measured on the chair: holding the stick left gives X = -96, so a ROS left turn
    # (positive angular.z) must encode as negative X.
    x_left, _ = frames.twist_to_joystick(0.0, 0.5, 0.5, 1.0, max_pct=20)
    x_right, _ = frames.twist_to_joystick(0.0, -0.5, 0.5, 1.0, max_pct=20)
    assert x_left == -10
    assert x_right == 10


@pytest.mark.req('6.3.2.2')
def test_forward_sends_positive_y():
    # Measured on the chair: holding the stick forward gives Y = +96.
    _, y_forward = frames.twist_to_joystick(0.25, 0.0, 0.5, 1.0, max_pct=20)
    _, y_reverse = frames.twist_to_joystick(-0.25, 0.0, 0.5, 1.0, max_pct=20)
    assert y_forward == 10
    assert y_reverse == -10


@pytest.mark.req('6.3.2.1', '6.3.2.2')
@pytest.mark.parametrize('data, linear_sign, angular_sign', [
    (bytes([0x00, 0x60]), 1, 0),    # chair JSM, stick forward: Y = +96
    (bytes([0xA0, 0x00]), 0, 1),    # chair JSM, stick left:    X = -96
])
def test_encoding_matches_chair_joystick_frames(data, linear_sign, angular_sign):
    jsm_x, jsm_y = frames.decode_joystick(data)
    x, y = frames.twist_to_joystick(0.5 * linear_sign, 1.0 * angular_sign, 0.5, 1.0, 100)
    assert (x > 0) == (jsm_x > 0) and (x < 0) == (jsm_x < 0)
    assert (y > 0) == (jsm_y > 0) and (y < 0) == (jsm_y < 0)


@pytest.mark.req('6.3.2.3', '6.3.6')
def test_command_beyond_limit_clamps_to_max_pct():
    assert frames.twist_to_joystick(5.0, -5.0, 0.5, 1.0, max_pct=20) == (20, 20)


@pytest.mark.req('6.3.6')
@pytest.mark.parametrize('max_pct', [-1, 101])
def test_max_pct_out_of_range_is_rejected(max_pct):
    with pytest.raises(ValueError):
        frames.twist_to_joystick(0.0, 0.0, 0.5, 1.0, max_pct=max_pct)


@pytest.mark.req('6.3.2')
def test_non_positive_limits_are_rejected():
    with pytest.raises(ValueError):
        frames.twist_to_joystick(0.0, 0.0, 0.0, 1.0)


# --- 6.4.1 status decoding -----------------------------------------------------------------

@pytest.mark.req('6.4.1')
def test_decode_pm_heartbeat():
    assert frames.decode_status(0x0C140000, b'', PM_NIBBLE) == {'pm_alive': True}


@pytest.mark.req('6.4.1')
def test_decode_motor_current_is_16_bit_little_endian():
    status = frames.decode_status(0x14300000, bytes([0x34, 0x12]), PM_NIBBLE)
    assert status == {'motor_current_a': 0x1234}


@pytest.mark.req('6.4.1', '6.4.4')
def test_decode_motor_current_applies_scale():
    status = frames.decode_status(0x14300000, bytes([100, 0]), PM_NIBBLE, current_scale=0.1)
    assert status['motor_current_a'] == pytest.approx(10.0)


@pytest.mark.req('6.4.1')
@pytest.mark.parametrize('raw, expected', [(0, 0), (55, 55), (100, 100), (200, 100)])
def test_decode_battery_pct_limited_to_100(raw, expected):
    assert frames.decode_status(0x1C0C0000, bytes([raw]), PM_NIBBLE) == {'battery_pct': expected}


@pytest.mark.req('6.4.1')
def test_decode_motor_events():
    assert frames.decode_status(0x0C000005, b'', PM_NIBBLE) == {
        'motors_stopped': True, 'motors_decelerating': False}
    assert frames.decode_status(0x0C000006, b'', PM_NIBBLE) == {
        'motors_stopped': False, 'motors_decelerating': True}


@pytest.mark.req('6.4.1', '6.2.1')
def test_status_from_other_device_nibble_is_not_decoded():
    assert frames.decode_status(0x0C140100, b'', PM_NIBBLE) is None


@pytest.mark.req('6.4.3')
@pytest.mark.parametrize('frame_id, data', [
    (0x123, bytes(8)),            # unknown ID
    (0x14300000, bytes([0x01])),  # current frame too short
    (0x1C0C0000, b''),            # battery frame empty
])
def test_undecodable_frames_return_none(frame_id, data):
    assert frames.decode_status(frame_id, data, PM_NIBBLE) is None
