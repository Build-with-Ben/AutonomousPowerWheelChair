"""L1 tests for device profiles (requirement 2.1.2)."""

from pathlib import Path

import pytest
from wheelchair_input.profile import (
    Axis, AxisButton, Button, ButtonAxis, ButtonCombo, DRIVE_AXES, from_dict, FUNCTIONS,
    load, NoAxis, NoButton, Profile, ProfileError,
)
import yaml


def write(tmp_path, doc):
    path = tmp_path / 'device.yaml'
    path.write_text(yaml.safe_dump(doc))
    return path


# --- analog axes -------------------------------------------------------------------------

@pytest.mark.req('2.1.2')
def test_axis_reads_its_configured_index():
    assert Axis(index=2).value([0.0, 0.0, 0.8], []) == pytest.approx(0.8)


@pytest.mark.req('2.1.2')
def test_axis_invert_flips_the_sign():
    assert Axis(index=0, invert=True).value([0.6], []) == pytest.approx(-0.6)


@pytest.mark.req('2.1.2')
def test_axis_trim_removes_a_resting_offset():
    """A stick that rests off centre is corrected in the profile, not in teleop."""
    assert Axis(index=0, trim=-0.05).value([-0.05], []) == pytest.approx(0.0)


@pytest.mark.req('2.1.2')
def test_axis_scale_can_expand_a_short_throw_device():
    assert Axis(index=0, scale=2.0).value([0.4], []) == pytest.approx(0.8)


@pytest.mark.req('2.1.2', '2.2.4')
def test_axis_output_is_clamped_to_the_normalised_range():
    """Scale or trim must never let a device demand more than full deflection."""
    assert Axis(index=0, scale=5.0).value([0.9], []) == pytest.approx(1.0)
    assert Axis(index=0, scale=5.0).value([-0.9], []) == pytest.approx(-1.0)


@pytest.mark.req('2.1.2')
def test_axis_beyond_the_message_reads_zero():
    """A profile for a 6-axis pad must not crash on a 2-axis device."""
    assert Axis(index=7).value([0.0, 0.0], []) == 0.0


# --- two buttons as one axis (a digital joystick) ----------------------------------------

@pytest.mark.req('2.1.2')
@pytest.mark.parametrize('pressed, expected', [
    ((), 0.0),
    ((3,), 1.0),
    ((5,), -1.0),
    ((3, 5), 0.0),      # both directions at once cancels, which is the safe reading
])
def test_button_axis_builds_an_axis_from_two_switches(pressed, expected):
    buttons = [1 if i in pressed else 0 for i in range(8)]
    assert ButtonAxis(plus=3, minus=5).value([], buttons) == pytest.approx(expected)


@pytest.mark.req('2.1.2')
def test_button_axis_with_unset_indices_reads_zero():
    assert ButtonAxis().value([], [1] * 8) == 0.0


# --- function sources --------------------------------------------------------------------

@pytest.mark.req('2.1.2')
def test_button_reads_its_configured_index():
    assert Button(index=4).held([], [0, 0, 0, 0, 1])
    assert not Button(index=4).held([], [1, 1, 1, 1, 0])


@pytest.mark.req('2.1.2')
def test_button_beyond_the_message_is_not_held():
    assert not Button(index=11).held([], [1, 1])


@pytest.mark.req('2.1.2', '2.3.5')
def test_combo_needs_every_button():
    combo = ButtonCombo(indices=(6, 7))
    assert combo.held([], [0, 0, 0, 0, 0, 0, 1, 1])
    assert not combo.held([], [0, 0, 0, 0, 0, 0, 1, 0])
    assert not combo.held([], [0] * 8)


@pytest.mark.req('2.1.2')
def test_axis_as_button_triggers_past_its_threshold():
    """An analog trigger used as a deadman."""
    trigger = AxisButton(index=2, threshold=0.5)
    assert trigger.held([0.0, 0.0, 0.7], [])
    assert not trigger.held([0.0, 0.0, 0.3], [])


@pytest.mark.req('2.1.2')
def test_axis_as_button_can_be_inverted_for_a_trigger_resting_high():
    trigger = AxisButton(index=2, threshold=0.5, invert=True)
    assert trigger.held([0.0, 0.0, -0.7], [])
    assert not trigger.held([0.0, 0.0, 1.0], [])


# --- unmapped controls fail safe ---------------------------------------------------------

@pytest.mark.req('2.1.2', '2.3.1')
def test_unmapped_controls_default_to_inactive():
    """An unmapped deadman must mean 'never drive', not 'always enabled'."""
    empty = Profile(name='empty')
    assert empty.axis('linear').value([1.0] * 8, [1] * 8) == 0.0
    assert not empty.function('deadman').held([1.0] * 8, [1] * 8)
    assert isinstance(empty.axis('linear'), NoAxis)
    assert isinstance(empty.function('deadman'), NoButton)


@pytest.mark.req('2.1.2')
def test_unmapped_lists_every_control_without_a_source():
    profile = from_dict({'name': 'partial', 'deadman': {'type': 'button', 'index': 0}})
    assert 'deadman' not in profile.unmapped
    assert set(profile.unmapped) == set(DRIVE_AXES + FUNCTIONS) - {'deadman'}


@pytest.mark.req('2.1.2')
def test_explicit_none_type_is_accepted():
    profile = from_dict({'name': 'n', 'angular': {'type': 'none'},
                         'estop': {'type': 'none'}})
    assert profile.axis('angular').value([1.0], [1]) == 0.0
    assert not profile.function('estop').held([1.0], [1])


# --- loading and validation --------------------------------------------------------------

@pytest.mark.req('2.1.2')
def test_profile_loads_from_a_yaml_file(tmp_path):
    path = write(tmp_path, {
        'name': 'bench',
        'description': 'a test pad',
        'linear': {'type': 'axis', 'index': 1, 'invert': True},
        'deadman': {'type': 'button', 'index': 6},
        'mode_manual': {'type': 'combo', 'indices': [8, 9]},
    })
    profile = load(path)
    assert profile.name == 'bench'
    assert profile.description == 'a test pad'
    assert profile.axes['linear'] == Axis(index=1, invert=True)
    assert profile.functions['deadman'] == Button(index=6)
    assert profile.functions['mode_manual'] == ButtonCombo(indices=(8, 9))


@pytest.mark.req('2.1.2')
def test_file_name_is_the_fallback_profile_name(tmp_path):
    path = write(tmp_path, {'deadman': {'type': 'button', 'index': 0}})
    assert load(path).name == 'device'


@pytest.mark.req('2.1.2')
@pytest.mark.parametrize('doc, expected', [
    ({'name': 'x', 'linear': {'type': 'axis'}}, 'needs index'),
    ({'name': 'x', 'deadman': {'type': 'combo'}}, 'needs indices'),
    ({'name': 'x', 'deadman': {'type': 'combo', 'indices': []}}, 'non-empty list'),
    ({'name': 'x', 'deadman': {'type': 'wiggle', 'index': 1}}, 'is not one of'),
    ({'name': 'x', 'linear': {'type': 'button', 'index': 1}}, 'is not one of'),
    ({'name': 'x', 'deadman': {'type': 'axis', 'plus': 1}}, 'has no setting'),
    ({'name': 'x', 'deadman': 4}, 'expected a mapping'),
    ({'name': 'x', 'wobble': {'type': 'button', 'index': 1}}, 'unknown key'),
    ({'linear': {'type': 'axis', 'index': 0}}, "needs a 'name'"),
])
def test_malformed_profile_is_rejected_with_a_useful_message(doc, expected):
    """A typo in a profile must fail loudly at startup, not drive the wrong axis."""
    with pytest.raises(ProfileError, match=expected):
        from_dict(doc)


@pytest.mark.req('2.1.2')
def test_missing_file_is_reported_as_a_profile_error(tmp_path):
    with pytest.raises(ProfileError, match='cannot read profile'):
        load(tmp_path / 'absent.yaml')


@pytest.mark.req('2.1.2')
def test_invalid_yaml_is_reported_as_a_profile_error(tmp_path):
    path = tmp_path / 'broken.yaml'
    path.write_text('name: x\n  bad indent: [\n')
    with pytest.raises(ProfileError, match='not valid YAML'):
        load(path)


@pytest.mark.req('2.1.2')
def test_empty_file_is_reported_as_a_profile_error(tmp_path):
    path = tmp_path / 'empty.yaml'
    path.write_text('')
    with pytest.raises(ProfileError, match='is empty'):
        load(path)


# --- the profiles this package ships -----------------------------------------------------

def shipped_profiles():
    """Return every profile YAML, from the install share or the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        config = Path(get_package_share_directory('wheelchair_input')) / 'config'
        found = sorted(config.glob('*.yaml'))
        if found:
            return found
    except Exception:                                       # noqa: BLE001
        pass    # not built yet; fall back to the source tree
    return sorted((Path(__file__).resolve().parents[1] / 'config').glob('*.yaml'))


@pytest.mark.req('2.1.2')
def test_shipped_profiles_exist():
    assert shipped_profiles(), 'no device profiles found under config/'


@pytest.mark.req('2.1.2')
def test_every_shipped_profile_loads_and_maps_a_deadman():
    """A shipped profile with no deadman could never drive the chair (2.3.1)."""
    for path in shipped_profiles():
        profile = load(path)
        assert profile.name == path.stem, f'{path.name}: name should match the file name'
        assert 'deadman' not in profile.unmapped, f'{path.name}: no deadman mapped'
        assert 'linear' not in profile.unmapped, f'{path.name}: no linear axis mapped'
