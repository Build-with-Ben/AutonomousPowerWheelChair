"""L1 tests for raw-device-to-operator-input mapping (requirement 2.1.2)."""

import pytest
from wheelchair_input.mapping import InputMapping, OperatorState
from wheelchair_input.profile import from_dict, FUNCTIONS, Profile

PAD = from_dict({
    'name': 'pad',
    'linear': {'type': 'axis', 'index': 1},
    'angular': {'type': 'axis', 'index': 0, 'invert': True},
    'deadman': {'type': 'button', 'index': 4},
    'estop': {'type': 'button', 'index': 1},
    'speed_up': {'type': 'button', 'index': 7},
    'speed_down': {'type': 'button', 'index': 6},
    'mode_manual': {'type': 'combo', 'indices': [8, 9]},
})

STICK = from_dict({
    'name': 'stick',
    'linear': {'type': 'buttons', 'plus': 0, 'minus': 1},
    'angular': {'type': 'buttons', 'plus': 2, 'minus': 3},
    'deadman': {'type': 'button', 'index': 4},
})


def buttons(*pressed, count=12):
    return [1 if i in pressed else 0 for i in range(count)]


def axes(**values):
    a = [0.0] * 6
    for index, value in values.items():
        a[int(index.lstrip('a'))] = value
    return a


# --- the mapping itself ------------------------------------------------------------------

@pytest.mark.req('2.1.2', '2.2.1.1', '2.2.1.2')
def test_profile_axes_reach_the_named_drive_fields():
    state = InputMapping(PAD).apply(axes(a0=0.5, a1=0.8), buttons())
    assert state.linear == pytest.approx(0.8)
    assert state.angular == pytest.approx(-0.5)      # 'angular' is inverted in PAD


@pytest.mark.req('2.1.2')
def test_profile_name_is_reported_as_the_source():
    """Teleop uses `source` to tell devices apart, so it must come from the profile."""
    assert InputMapping(PAD).apply(axes(), buttons()).source == 'pad'
    assert InputMapping(STICK).apply(axes(), buttons()).source == 'stick'


@pytest.mark.req('2.1.2', '2.3.1')
def test_each_function_is_set_from_its_own_control():
    mapping = InputMapping(PAD)
    assert mapping.apply(axes(), buttons(4)).deadman
    assert mapping.apply(axes(), buttons(1)).estop
    assert mapping.apply(axes(), buttons(7)).speed_up
    assert mapping.apply(axes(), buttons(6)).speed_down
    assert mapping.apply(axes(), buttons(8, 9)).mode_manual
    assert not mapping.apply(axes(), buttons(8)).mode_manual


@pytest.mark.req('2.1.2')
def test_unrelated_buttons_set_no_function():
    state = InputMapping(PAD).apply(axes(), buttons(0, 2, 3, 5, 10, 11))
    assert not any(getattr(state, name) for name in FUNCTIONS)


@pytest.mark.req('2.1.2')
def test_two_profiles_give_different_meanings_to_the_same_raw_message():
    """The whole point of the layer: the device changes, teleop's input does not."""
    raw_axes, raw_buttons = axes(a1=1.0), buttons(0, 4)
    assert InputMapping(PAD).apply(raw_axes, raw_buttons).linear == pytest.approx(1.0)
    # The digital stick ignores the analog axis and reads button 0 as forward instead.
    assert InputMapping(STICK).apply(raw_axes, raw_buttons).linear == pytest.approx(1.0)
    assert InputMapping(STICK).apply(axes(), buttons(4)).linear == pytest.approx(0.0)


@pytest.mark.req('2.1.2')
def test_digital_joystick_produces_full_deflection_from_switches():
    state = InputMapping(STICK).apply(axes(), buttons(0, 2, 4))
    assert (state.linear, state.angular) == pytest.approx((1.0, 1.0))
    assert state.deadman


# --- robustness against a device that does not match its profile -------------------------

@pytest.mark.req('2.1.2')
def test_short_message_does_not_raise():
    """A pad with fewer axes or buttons than the profile expects must not crash the node."""
    state = InputMapping(PAD).apply([0.0], [0])
    assert (state.linear, state.angular) == pytest.approx((0.0, 0.0))
    assert not state.deadman


@pytest.mark.req('2.1.2')
def test_empty_message_does_not_raise():
    state = InputMapping(PAD).apply([], [])
    assert state.idle


@pytest.mark.req('2.1.2', '2.3.1')
def test_empty_profile_never_reports_a_deadman():
    """A profile that maps nothing must leave the chair unable to move."""
    state = InputMapping(Profile(name='empty')).apply(axes(a1=1.0), buttons(*range(12)))
    assert state.idle
    assert not state.deadman


@pytest.mark.req('2.1.2')
def test_mapping_is_stateless():
    """Each message is mapped on its own, so a profile swap carries no stale state."""
    mapping = InputMapping(PAD)
    assert mapping.apply(axes(a1=1.0), buttons(4)).deadman
    assert not mapping.apply(axes(), buttons()).deadman
    assert mapping.apply(axes(), buttons()) == OperatorState(source='pad')


@pytest.mark.req('2.1.2')
def test_output_fields_have_plain_python_types():
    """The node assigns these straight onto a ROS message, which rejects numpy scalars."""
    state = InputMapping(PAD).apply(axes(a1=0.5), buttons(4))
    assert type(state.linear) is float
    assert type(state.angular) is float
    for name in FUNCTIONS:
        assert type(getattr(state, name)) is bool


@pytest.mark.req('2.1.2')
def test_idle_is_false_when_any_function_is_held():
    assert not InputMapping(PAD).apply(axes(), buttons(4)).idle
    assert InputMapping(PAD).apply(axes(), buttons()).idle
