"""L1 tests for teleop gamepad logic (domain 2)."""

import pytest
from wheelchair_teleop.teleop_logic import apply_deadzone, Teleop, TeleopConfig


CFG = TeleopConfig(speed_scales=(0.25, 0.5, 1.0))
N_BUTTONS = 12


def buttons(*pressed):
    return [1 if i in pressed else 0 for i in range(N_BUTTONS)]


def axes(linear=0.0, angular=0.0):
    a = [0.0] * 8
    a[CFG.linear_axis] = linear
    a[CFG.angular_axis] = angular
    return a


def full_speed_teleop():
    t = Teleop(CFG)
    for _ in range(len(CFG.speed_scales)):
        t.on_joy(axes(), buttons(CFG.speed_up_button), now=0.0)
        t.on_joy(axes(), buttons(), now=0.0)
    return t


# --- 2.2.3 deadzone ----------------------------------------------------------------------

@pytest.mark.req('2.2.3')
@pytest.mark.parametrize('value', [0.0, 0.05, -0.05, 0.1, -0.1])
def test_inside_deadzone_is_zero(value):
    assert apply_deadzone(value, 0.1) == 0.0


@pytest.mark.req('2.2.3', '2.2.4')
@pytest.mark.parametrize('value, expected', [(1.0, 1.0), (-1.0, -1.0), (0.55, 0.5)])
def test_outside_deadzone_rescaled(value, expected):
    assert apply_deadzone(value, 0.1) == pytest.approx(expected)


@pytest.mark.req('2.2.3')
def test_deadzone_output_is_continuous_at_edge():
    assert apply_deadzone(0.1001, 0.1) == pytest.approx(0.0, abs=1e-3)


# --- 2.2 command generation ----------------------------------------------------------------

@pytest.mark.req('2.2.1.1', '2.2.1.2', '2.2.4')
def test_full_deflection_maps_to_max_speeds():
    t = full_speed_teleop()
    t.on_joy(axes(linear=1.0, angular=-1.0), buttons(CFG.deadman_button), now=1.0)
    assert t.tick(now=1.0) == pytest.approx((0.5, -1.0))


@pytest.mark.req('2.2.1.1', '2.2.1.2')
def test_axes_map_to_their_own_twist_fields():
    t = full_speed_teleop()
    t.on_joy(axes(linear=1.0), buttons(CFG.deadman_button), now=1.0)
    assert t.tick(now=1.0) == pytest.approx((0.5, 0.0))
    t.on_joy(axes(angular=1.0), buttons(CFG.deadman_button), now=1.0)
    assert t.tick(now=1.0) == pytest.approx((0.0, 1.0))


@pytest.mark.req('2.2.1.3')
def test_output_has_only_linear_x_and_angular_z():
    t = full_speed_teleop()
    t.on_joy(axes(linear=0.7, angular=0.3), buttons(CFG.deadman_button), now=1.0)
    assert len(t.tick(now=1.0)) == 2


@pytest.mark.req('2.1.2')
def test_mapping_comes_from_config():
    cfg = TeleopConfig(linear_axis=3, angular_axis=2, deadman_button=0, speed_scales=(1.0,))
    t = Teleop(cfg)
    a = [0.0] * 8
    a[3], a[2] = 1.0, 0.0
    t.on_joy(a, [1] + [0] * 11, now=0.0)
    assert t.tick(now=0.0) == pytest.approx((0.5, 0.0))


@pytest.mark.req('2.2.2')
def test_publishes_every_tick_while_deadman_held_even_when_centered():
    t = Teleop(CFG)
    out = []
    for i in range(10):
        now = i * 0.05
        t.on_joy(axes(), buttons(CFG.deadman_button), now=now)
        out.append(t.tick(now=now))
    assert out == [(0.0, 0.0)] * 10


@pytest.mark.req('2.2.5')
def test_speed_scale_starts_at_lowest_step():
    t = Teleop(CFG)
    assert t.speed_step == 0
    t.on_joy(axes(linear=1.0), buttons(CFG.deadman_button), now=0.0)
    assert t.tick(now=0.0)[0] == pytest.approx(0.5 * 0.25)


@pytest.mark.req('2.2.5')
def test_speed_scale_has_three_steps_and_saturates():
    assert len(CFG.speed_scales) >= 3
    t = Teleop(CFG)
    for _ in range(5):
        t.on_joy(axes(), buttons(CFG.speed_up_button), now=0.0)
        t.on_joy(axes(), buttons(), now=0.0)
    assert t.speed_step == 2
    for _ in range(5):
        t.on_joy(axes(), buttons(CFG.speed_down_button), now=0.0)
        t.on_joy(axes(), buttons(), now=0.0)
    assert t.speed_step == 0


@pytest.mark.req('2.2.5')
def test_holding_speed_button_changes_one_step():
    t = Teleop(CFG)
    for _ in range(5):
        t.on_joy(axes(), buttons(CFG.speed_up_button), now=0.0)
    assert t.speed_step == 1


# --- 2.3 deadman and stop ----------------------------------------------------------------

@pytest.mark.req('2.3.1', '1.4.2')
def test_no_output_without_deadman():
    t = Teleop(CFG)
    t.on_joy(axes(linear=1.0, angular=1.0), buttons(), now=0.0)
    assert t.tick(now=0.0) is None


@pytest.mark.req('2.3.1', '1.4.2')
def test_no_output_before_first_joy_message():
    assert Teleop(CFG).tick(now=0.0) is None


@pytest.mark.req('2.3.2')
def test_release_publishes_one_zero_then_stops():
    t = Teleop(CFG)
    t.on_joy(axes(linear=1.0), buttons(CFG.deadman_button), now=0.0)
    assert t.tick(now=0.0) is not None
    t.on_joy(axes(linear=1.0), buttons(), now=0.05)
    assert t.tick(now=0.05) == (0.0, 0.0)
    assert t.tick(now=0.10) is None
    assert t.tick(now=0.15) is None


@pytest.mark.req('2.1.3')
def test_disconnect_detected_after_timeout():
    t = Teleop(CFG)
    t.on_joy(axes(), buttons(), now=0.0)
    assert t.connected(now=0.5)
    assert not t.connected(now=0.51)


@pytest.mark.req('2.3.3', '1.4.1')
def test_disconnect_stops_publishing_without_zero():
    t = Teleop(CFG)
    t.on_joy(axes(linear=1.0), buttons(CFG.deadman_button), now=0.0)
    assert t.tick(now=0.4) is not None
    assert t.tick(now=0.6) is None
    assert t.tick(now=0.7) is None


@pytest.mark.req('2.3.3')
def test_reconnect_with_deadman_released_publishes_nothing():
    t = Teleop(CFG)
    t.on_joy(axes(linear=1.0), buttons(CFG.deadman_button), now=0.0)
    t.tick(now=0.0)
    assert t.tick(now=1.0) is None
    t.on_joy(axes(), buttons(), now=1.0)
    assert t.tick(now=1.0) is None


@pytest.mark.req('2.3.4')
def test_estop_button_raises_estop_once_per_press():
    t = Teleop(CFG)
    assert t.on_joy(axes(), buttons(CFG.estop_button), now=0.0).estop
    assert not t.on_joy(axes(), buttons(CFG.estop_button), now=0.05).estop
    assert not t.on_joy(axes(), buttons(), now=0.10).estop
    assert t.on_joy(axes(), buttons(CFG.estop_button), now=0.15).estop


@pytest.mark.req('2.3.5')
def test_button_combo_requests_manual_mode():
    t = Teleop(CFG)
    first, second = CFG.manual_mode_buttons
    assert not t.on_joy(axes(), buttons(first), now=0.0).manual_mode
    assert t.on_joy(axes(), buttons(first, second), now=0.05).manual_mode
    assert not t.on_joy(axes(), buttons(first, second), now=0.10).manual_mode
