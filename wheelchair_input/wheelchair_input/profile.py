"""
Device profiles: how a controller's axes and buttons drive operator functions (2.1.2).

Pure logic, no rclpy and no ROS messages. A profile is a YAML document describing one
physical device; nothing else in the stack knows an axis or button index. Swapping
controllers means loading a different profile, not changing code.

Each logical control is backed by a *source*, chosen by its ``type`` in the YAML:

Drive axes (``linear``, ``angular``) produce -1.0 .. +1.0:
    ``axis``     an analog axis, with ``index``, ``invert``, ``scale`` and ``trim``
    ``buttons``  two buttons as one axis, with ``plus`` and ``minus`` (a D-pad, or the
                 four direction switches of a dumb digital joystick)
    ``none``     always 0.0

Functions (``deadman``, ``estop``, ``speed_up``, ``speed_down``, ``mode_manual``,
``rearm``) produce a held/not-held boolean:
    ``button``   one button, by ``index``
    ``combo``    every button in ``indices`` held together
    ``axis``     an analog axis past ``threshold``, for a trigger used as a button
    ``none``     always False

``none`` is the default for every control, so an unmapped function fails safe: an
unmapped ``deadman`` means the chair never moves, rather than always being enabled.
"""

from dataclasses import dataclass, field, fields, MISSING
from pathlib import Path

import yaml

#: Drive axes every profile may define, in message field order.
DRIVE_AXES = ('linear', 'angular')

#: Boolean operator functions every profile may define, in message field order.
FUNCTIONS = ('deadman', 'estop', 'speed_up', 'speed_down', 'mode_manual', 'rearm')


class ProfileError(ValueError):
    """A profile document is malformed, so no device mapping can be built from it."""


def _at(seq, index, default=0.0):
    """
    Return ``seq[index]``, or ``default`` when the index is unset or out of range.

    A short ``/joy`` message or an index left at its -1 default reads as the default
    rather than raising, so a profile written for a 13-button pad does not crash the
    node when a 6-button pad is plugged in.
    """
    return seq[index] if 0 <= index < len(seq) else default


# --- drive axis sources ---------------------------------------------------------------

@dataclass(frozen=True)
class NoAxis:
    """An unmapped drive axis. Always 0.0."""

    def value(self, axes, buttons):
        """Return the normalised axis value for one raw device message."""
        return 0.0


@dataclass(frozen=True)
class Axis:
    """An analog axis, corrected to -1.0 .. +1.0 with positive in the ROS direction."""

    index: int
    invert: bool = False
    scale: float = 1.0
    trim: float = 0.0       # resting value to subtract, for a stick that does not rest at 0

    def value(self, axes, buttons):
        """Return the normalised axis value for one raw device message."""
        raw = (_at(axes, self.index) - self.trim) * self.scale
        if self.invert:
            raw = -raw
        return max(-1.0, min(1.0, raw))


@dataclass(frozen=True)
class ButtonAxis:
    """Two buttons as one axis: a D-pad, or a digital joystick's direction switches."""

    plus: int = -1
    minus: int = -1

    def value(self, axes, buttons):
        """Return the normalised axis value for one raw device message."""
        up = 1.0 if _at(buttons, self.plus, 0) else 0.0
        down = 1.0 if _at(buttons, self.minus, 0) else 0.0
        return up - down       # both held cancels out, which is the safe reading


# --- function sources -----------------------------------------------------------------

@dataclass(frozen=True)
class NoButton:
    """An unmapped function. Always False, so the function is simply unavailable."""

    def held(self, axes, buttons):
        """Return whether the control is held for one raw device message."""
        return False


@dataclass(frozen=True)
class Button:
    """One button, by index."""

    index: int

    def held(self, axes, buttons):
        """Return whether the control is held for one raw device message."""
        return bool(_at(buttons, self.index, 0))


@dataclass(frozen=True)
class ButtonCombo:
    """Several buttons that must be held together (2.3.5)."""

    indices: tuple

    def held(self, axes, buttons):
        """Return whether the control is held for one raw device message."""
        return bool(self.indices) and all(_at(buttons, i, 0) for i in self.indices)


@dataclass(frozen=True)
class AxisButton:
    """An analog axis used as a button, such as a trigger held past a threshold."""

    index: int
    threshold: float = 0.5
    invert: bool = False    # for a trigger that rests at +1 and falls towards -1

    def held(self, axes, buttons):
        """Return whether the control is held for one raw device message."""
        value = _at(axes, self.index)
        return (-value if self.invert else value) >= self.threshold


#: ``type`` keyword -> source class, per kind of control.
AXIS_TYPES = {'axis': Axis, 'buttons': ButtonAxis, 'none': NoAxis}
FUNCTION_TYPES = {'button': Button, 'combo': ButtonCombo, 'axis': AxisButton,
                  'none': NoButton}


@dataclass(frozen=True)
class Profile:
    """One controller's complete mapping, named so it can be reported in `source`."""

    name: str
    description: str = ''
    axes: dict = field(default_factory=dict)        # 'linear'/'angular' -> axis source
    functions: dict = field(default_factory=dict)   # function name -> function source

    def axis(self, name):
        """Return the source for a drive axis, or ``NoAxis`` when it is unmapped."""
        return self.axes.get(name, NoAxis())

    def function(self, name):
        """Return the source for a function, or ``NoButton`` when it is unmapped."""
        return self.functions.get(name, NoButton())

    @property
    def unmapped(self):
        """Return the names of controls with no source, worth warning about at startup."""
        return tuple(n for n in DRIVE_AXES + FUNCTIONS
                     if isinstance(self.axis(n) if n in DRIVE_AXES else self.function(n),
                                   (NoAxis, NoButton)))


def _build(control, spec, types):
    """Build one source from its YAML mapping, rejecting anything the class cannot take."""
    if not isinstance(spec, dict):
        raise ProfileError(
            f'{control!r}: expected a mapping such as '
            f'{{type: button, index: 3}}, got {type(spec).__name__}')
    spec = dict(spec)
    kind = spec.pop('type', None)
    if kind not in types:
        raise ProfileError(
            f'{control!r}: type {kind!r} is not one of {sorted(types)}')
    cls = types[kind]
    allowed = {f.name for f in fields(cls)}
    unknown = sorted(set(spec) - allowed)
    if unknown:
        raise ProfileError(
            f'{control!r}: type {kind!r} has no setting(s) '
            f'{", ".join(unknown)}; it takes {sorted(allowed)}')
    required = {f.name for f in fields(cls)
                if f.default is MISSING and f.default_factory is MISSING}
    missing = sorted(required - set(spec))
    if missing:
        raise ProfileError(
            f'{control!r}: type {kind!r} needs {", ".join(missing)}')
    if 'indices' in spec:
        if not isinstance(spec['indices'], (list, tuple)) or not spec['indices']:
            raise ProfileError(f"{control!r}: 'indices' must be a non-empty list")
        spec['indices'] = tuple(spec['indices'])
    return cls(**spec)


def from_dict(doc, name=None):
    """Build a :class:`Profile` from a parsed YAML document (2.1.2)."""
    if not isinstance(doc, dict):
        raise ProfileError(f'profile must be a mapping, got {type(doc).__name__}')
    doc = dict(doc)
    profile_name = doc.pop('name', None) or name
    if not profile_name:
        raise ProfileError("profile needs a 'name'")
    description = doc.pop('description', '') or ''

    axes, functions = {}, {}
    for control in DRIVE_AXES:
        if control in doc:
            axes[control] = _build(control, doc.pop(control), AXIS_TYPES)
    for control in FUNCTIONS:
        if control in doc:
            functions[control] = _build(control, doc.pop(control), FUNCTION_TYPES)

    unknown = sorted(doc)
    if unknown:
        allowed = ', '.join(('name', 'description') + DRIVE_AXES + FUNCTIONS)
        raise ProfileError(
            f'unknown key(s) {", ".join(unknown)}; '
            f'a profile may set {allowed}')
    return Profile(name=profile_name, description=description, axes=axes,
                   functions=functions)


def load(path):
    """Load a profile from a YAML file. The file name is the fallback profile name."""
    path = Path(path)
    try:
        doc = yaml.safe_load(path.read_text())
    except OSError as exc:
        raise ProfileError(f'cannot read profile {path}: {exc}') from exc
    except yaml.YAMLError as exc:
        raise ProfileError(f'{path} is not valid YAML: {exc}') from exc
    if doc is None:
        raise ProfileError(f'{path} is empty')
    try:
        return from_dict(doc, name=path.stem)
    except ProfileError as exc:
        raise ProfileError(f'{path}: {exc}') from exc
