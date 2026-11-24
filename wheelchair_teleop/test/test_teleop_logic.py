"""L1 tests for teleop operator-input logic (domain 2)."""

from dataclasses import dataclass

import pytest
from wheelchair_teleop.teleop_logic import apply_deadzone, Teleop, TeleopConfig

CFG = TeleopConfig(speed_scales=(0.25, 0.5, 1.0))


@dataclass
class Input:
    """Stands in for ``wheelchair_msgs/OperatorInput``, which needs no ROS to construct."""

    linear: float = 0.0
    angular: float = 0.0
    deadman: bool = False
    estop: bool = False
    speed_up: bool = False
    speed_down: bool = False
    mode_manual: bool = False
    source: str = 'pad'


def driving(**kwargs):
    """Return an input message with the deadman held."""
    return Input(deadman=True, **kwargs)


def full_speed_teleop(config=CFG):
    """Return a teleop at its highest speed step, so scaling cannot mask the maths."""
    t = Teleop(config)
    for _ in range(len(config.speed_scales)):
        t.on_input(Input(speed_up=True), now=0.0)
        t.on_input(Input(), now=0.0)
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


@pytest.mark.req('2.2.3')
def test_deadzone_applies_to_the_mapped_input():
    t = full_speed_teleop()
    t.on_input(driving(linear=0.05, angular=-0.05), now=1.0)
    assert t.tick(now=1.0) == pytest.approx((0.0, 0.0))


# --- 2.2 command generation --------------------------------------------------------------

@pytest.mark.req('2.2.1.1', '2.2.1.2', '2.2.4')
def test_full_deflection_maps_to_max_speeds():
    t = full_speed_teleop()
    t.on_input(driving(linear=1.0, angular=-1.0), now=1.0)
    assert t.tick(now=1.0) == pytest.approx((0.5, -1.0))


@pytest.mark.req('2.2.1.1', '2.2.1.2')
def test_each_input_field_maps_to_its_own_twist_field():
    t = full_speed_teleop()
    t.on_input(driving(linear=1.0), now=1.0)
    assert t.tick(now=1.0) == pytest.approx((0.5, 0.0))
    t.on_input(driving(angular=1.0), now=1.0)
    assert t.tick(now=1.0) == pytest.approx((0.0, 1.0))


@pytest.mark.req('2.2.1.3')
def test_output_has_only_linear_x_and_angular_z():
    t = full_speed_teleop()
    t.on_input(driving(linear=0.7, angular=0.3), now=1.0)
    assert len(t.tick(now=1.0)) == 2


@pytest.mark.req('2.2.4')
def test_max_speeds_come_from_config():
    t = full_speed_teleop(TeleopConfig(max_linear=0.2, max_angular=0.4,
                                       speed_scales=(1.0,)))
    t.on_input(driving(linear=1.0, angular=1.0), now=0.0)
    assert t.tick(now=0.0) == pytest.approx((0.2, 0.4))


@pytest.mark.req('2.2.2')
def test_publishes_every_tick_while_deadman_held_even_when_centered():
    t = Teleop(CFG)
    out = []
    for i in range(10):
        now = i * 0.05
        t.on_input(driving(), now=now)
        out.append(t.tick(now=now))
    assert out == [(0.0, 0.0)] * 10


@pytest.mark.req('2.2.5')
def test_speed_scale_starts_at_lowest_step():
    t = Teleop(CFG)
    assert t.speed_step == 0
    t.on_input(driving(linear=1.0), now=0.0)
    assert t.tick(now=0.0)[0] == pytest.approx(0.5 * 0.25)


@pytest.mark.req('2.2.5')
def test_speed_scale_has_three_steps_and_saturates():
    assert len(CFG.speed_scales) >= 3
    t = Teleop(CFG)
    for _ in range(5):
        t.on_input(Input(speed_up=True), now=0.0)
        t.on_input(Input(), now=0.0)
    assert t.speed_step == 2
    for _ in range(5):
        t.on_input(Input(speed_down=True), now=0.0)
        t.on_input(Input(), now=0.0)
    assert t.speed_step == 0


@pytest.mark.req('2.2.5')
def test_holding_speed_button_changes_one_step():
    t = Teleop(CFG)
    for _ in range(5):
        t.on_input(Input(speed_up=True), now=0.0)
    assert t.speed_step == 1


# --- 2.3 deadman and stop ----------------------------------------------------------------

@pytest.mark.req('2.3.1', '1.4.2')
def test_no_output_without_deadman():
    t = Teleop(CFG)
    t.on_input(Input(linear=1.0, angular=1.0), now=0.0)
    assert t.tick(now=0.0) is None


@pytest.mark.req('2.3.1', '1.4.2')
def test_no_output_before_first_input_message():
    assert Teleop(CFG).tick(now=0.0) is None


@pytest.mark.req('2.3.2')
def test_release_publishes_one_zero_then_stops():
    t = Teleop(CFG)
    t.on_input(driving(linear=1.0), now=0.0)
    assert t.tick(now=0.0) is not None
    t.on_input(Input(linear=1.0), now=0.05)
    assert t.tick(now=0.05) == (0.0, 0.0)
    assert t.tick(now=0.10) is None
    assert t.tick(now=0.15) is None


@pytest.mark.req('2.1.3')
def test_disconnect_detected_after_timeout():
    t = Teleop(CFG)
    t.on_input(Input(), now=0.0)
    assert t.connected(now=0.5)
    assert not t.connected(now=0.51)


@pytest.mark.req('2.3.3', '1.4.1')
def test_disconnect_stops_publishing_without_zero():
    t = Teleop(CFG)
    t.on_input(driving(linear=1.0), now=0.0)
    assert t.tick(now=0.4) is not None
    assert t.tick(now=0.6) is None
    assert t.tick(now=0.7) is None


@pytest.mark.req('2.3.3')
def test_reconnect_with_deadman_released_publishes_nothing():
    t = Teleop(CFG)
    t.on_input(driving(linear=1.0), now=0.0)
    t.tick(now=0.0)
    assert t.tick(now=1.0) is None
    t.on_input(Input(), now=1.0)
    assert t.tick(now=1.0) is None


@pytest.mark.req('2.3.4')
def test_estop_raises_estop_once_per_press():
    t = Teleop(CFG)
    assert t.on_input(Input(estop=True), now=0.0).estop
    assert not t.on_input(Input(estop=True), now=0.05).estop
    assert not t.on_input(Input(), now=0.10).estop
    assert t.on_input(Input(estop=True), now=0.15).estop


@pytest.mark.req('2.3.5')
def test_mode_manual_requests_manual_mode_once_per_press():
    t = Teleop(CFG)
    assert t.on_input(Input(mode_manual=True), now=0.0).manual_mode
    assert not t.on_input(Input(mode_manual=True), now=0.05).manual_mode
    assert not t.on_input(Input(), now=0.10).manual_mode
    assert t.on_input(Input(mode_manual=True), now=0.15).manual_mode


# --- 2.1.2 device independence and the enable/disable seam -------------------------------

@pytest.mark.req('2.1.2')
def test_teleop_needs_no_axis_or_button_indices():
    """Device layout belongs to wheelchair_input's profiles, not to teleop's config."""
    names = set(TeleopConfig().__dataclass_fields__)
    assert not {n for n in names if 'axis' in n or 'button' in n}


@pytest.mark.req('2.1.2')
def test_any_source_is_accepted_by_default():
    t = Teleop(CFG)
    assert t.accepts('xbox_generic') and t.accepts('atari_remote') and t.accepts('')


@pytest.mark.req('2.1.2')
def test_only_listed_sources_are_accepted_when_configured():
    t = Teleop(TeleopConfig(sources=('atari_remote',), speed_scales=(1.0,)))
    assert t.accepts('atari_remote')
    assert not t.accepts('xbox_generic')


@pytest.mark.req('2.1.2', '2.3.1')
def test_input_from_an_unaccepted_source_is_ignored_entirely():
    """A disabled controller must neither drive nor keep teleop's timeout alive."""
    t = Teleop(TeleopConfig(sources=('atari_remote',), speed_scales=(1.0,)))
    t.on_input(Input(linear=1.0, deadman=True, source='xbox_generic'), now=0.0)
    assert not t.connected(now=0.0)
    assert t.tick(now=0.0) is None


@pytest.mark.req('2.1.2')
def test_unaccepted_source_cannot_raise_estop_or_change_speed():
    t = Teleop(TeleopConfig(sources=('atari_remote',), speed_scales=(0.5, 1.0)))
    events = t.on_input(
        Input(estop=True, mode_manual=True, speed_up=True, source='other'), now=0.0)
    assert not events.estop and not events.manual_mode
    assert t.speed_step == 0


@pytest.mark.req('2.1.2')
def test_active_source_is_reported():
    t = Teleop(CFG)
    t.on_input(Input(source='atari_remote'), now=0.0)
    assert t.source == 'atari_remote'
