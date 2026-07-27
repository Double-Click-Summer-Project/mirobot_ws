"""Validation, conversion, and parsing for the Mirobot G-code protocol."""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Optional, Sequence


JOINT_AXES = ("X", "Y", "Z", "A", "B", "C")

_STATE_RE = re.compile(r"^\s*<\s*([^,>]+)", re.IGNORECASE)
_ANGLE_RE = re.compile(
    r"Angle\(ABCDXYZ\)\s*:\s*(.*?)\s*,\s*Cartesian\s+coordinate",
    re.IGNORECASE,
)
_CARTESIAN_RE = re.compile(
    r"Cartesian\s+coordinate\(XYZ\s+RxRyRz\)\s*:\s*"
    r"(.*?)\s*,\s*(?:Pump|Value|Valve)\s+PWM",
    re.IGNORECASE,
)


class CommandValidationError(ValueError):
    """A ROS command cannot be converted into a safe Mirobot command."""


@dataclass(frozen=True)
class ParsedStatus:
    """Fields that can be extracted from a Mirobot ``?`` response."""

    state: str
    raw: str
    joint_degrees: Optional[tuple[float, float, float, float, float, float]]
    xyz_mm: Optional[tuple[float, float, float]]
    complete: bool


def _finite_values(values: Sequence[float], label: str) -> list[float]:
    converted = [float(value) for value in values]
    if not all(math.isfinite(value) for value in converted):
        raise CommandValidationError(f"{label} contains NaN or infinity")
    return converted


def _positive_feedrate(value: int, label: str) -> int:
    feedrate = int(value)
    if feedrate <= 0:
        raise CommandValidationError(f"{label} must be greater than zero")
    return feedrate


def _format_number(value: float) -> str:
    if abs(value) < 0.0005:
        value = 0.0
    return f"{value:.2f}"


def ordered_joint_positions(
    names: Sequence[str],
    positions: Sequence[float],
    expected_names: Sequence[str],
) -> list[float]:
    """Return six joint positions in ``expected_names`` order."""

    if len(expected_names) != 6 or len(set(expected_names)) != 6:
        raise CommandValidationError(
            "joint_names parameter must contain six unique names"
        )

    if names:
        if len(names) != len(positions):
            raise CommandValidationError(
                "JointState name and position arrays must have equal lengths"
            )
        if len(set(names)) != len(names):
            raise CommandValidationError("JointState contains duplicate names")
        by_name = dict(zip(names, positions))
        missing = [name for name in expected_names if name not in by_name]
        if missing:
            raise CommandValidationError(
                "JointState is missing: " + ", ".join(missing)
            )
        ordered = [by_name[name] for name in expected_names]
    else:
        if len(positions) != 6:
            raise CommandValidationError(
                "JointState without names must contain exactly six positions"
            )
        ordered = list(positions)

    return _finite_values(ordered, "joint positions")


def make_joint_command(
    names: Sequence[str],
    positions_rad: Sequence[float],
    expected_names: Sequence[str],
    lower_limits_deg: Sequence[float],
    upper_limits_deg: Sequence[float],
    feedrate: int,
) -> str:
    """Convert a ROS JointState payload in rad to an absolute M21 command."""

    ordered_rad = ordered_joint_positions(
        names,
        positions_rad,
        expected_names,
    )
    lower = _finite_values(lower_limits_deg, "joint lower limits")
    upper = _finite_values(upper_limits_deg, "joint upper limits")
    if len(lower) != 6 or len(upper) != 6:
        raise CommandValidationError(
            "joint limit parameters must each contain six values"
        )

    degrees = [math.degrees(value) for value in ordered_rad]
    for index, (value, low, high) in enumerate(
        zip(degrees, lower, upper),
        start=1,
    ):
        if low > high:
            raise CommandValidationError(
                f"joint{index} lower limit is greater than its upper limit"
            )
        if not low <= value <= high:
            raise CommandValidationError(
                f"joint{index}={value:.2f} deg is outside "
                f"[{low:.2f}, {high:.2f}] deg"
            )

    fields = " ".join(
        f"{axis}{_format_number(value)}"
        for axis, value in zip(JOINT_AXES, degrees)
    )
    return (
        f"M21 G90 {fields} "
        f"F{_positive_feedrate(feedrate, 'joint_feedrate')}"
    )


def make_xyz_command(
    xyz_m: Sequence[float],
    lower_limits_mm: Sequence[float],
    upper_limits_mm: Sequence[float],
    feedrate: int,
    motion_mode: str,
) -> str:
    """Convert a ROS XYZ point in m to an absolute M20 command in mm."""

    xyz = _finite_values(xyz_m, "XYZ position")
    lower = _finite_values(lower_limits_mm, "XYZ lower limits")
    upper = _finite_values(upper_limits_mm, "XYZ upper limits")
    if len(xyz) != 3 or len(lower) != 3 or len(upper) != 3:
        raise CommandValidationError(
            "XYZ position and limit parameters must each contain three values"
        )

    xyz_mm = [value * 1000.0 for value in xyz]
    labels = ("x", "y", "z")
    for label, value, low, high in zip(labels, xyz_mm, lower, upper):
        if low > high:
            raise CommandValidationError(
                f"{label} lower limit is greater than its upper limit"
            )
        if not low <= value <= high:
            raise CommandValidationError(
                f"{label}={value:.2f} mm is outside "
                f"[{low:.2f}, {high:.2f}] mm"
            )

    mode = str(motion_mode).strip().upper()
    if mode not in {"G0", "G1"}:
        raise CommandValidationError(
            "cartesian_motion_mode must be either G0 or G1"
        )

    fields = " ".join(
        f"{axis}{_format_number(value)}"
        for axis, value in zip(("X", "Y", "Z"), xyz_mm)
    )
    return (
        f"M20 G90 {mode} {fields} "
        f"F{_positive_feedrate(feedrate, 'xyz_feedrate')}"
    )


def make_pump_command(pwm: int) -> str:
    """Create a bounded Mirobot vacuum-pump PWM command."""

    if isinstance(pwm, bool):
        raise CommandValidationError("pump PWM must be an integer")
    value = int(pwm)
    if value != pwm:
        raise CommandValidationError("pump PWM must be an integer")
    if not 0 <= value <= 1000:
        raise CommandValidationError(
            "pump PWM must be between 0 and 1000"
        )
    return f"M3S{value}"


def _parse_csv_numbers(text: str) -> Optional[list[float]]:
    try:
        values = [float(item.strip()) for item in text.split(",")]
    except ValueError:
        return None
    if not values or not all(math.isfinite(value) for value in values):
        return None
    return values


def parse_status(line: str) -> Optional[ParsedStatus]:
    """Parse status while tolerating firmware suffix variants."""

    raw = str(line).strip()
    state_match = _STATE_RE.search(raw)
    if state_match is None:
        return None

    joint_degrees = None
    angle_match = _ANGLE_RE.search(raw)
    if angle_match is not None:
        angles = _parse_csv_numbers(angle_match.group(1))
        if angles is not None:
            if len(angles) >= 7:
                # Firmware order: A, B, C, D (rail), X, Y, Z.
                joint_degrees = (
                    angles[4],
                    angles[5],
                    angles[6],
                    angles[0],
                    angles[1],
                    angles[2],
                )
            elif len(angles) == 6:
                # Some controllers omit the external rail D value.
                joint_degrees = (
                    angles[3],
                    angles[4],
                    angles[5],
                    angles[0],
                    angles[1],
                    angles[2],
                )

    xyz_mm = None
    cartesian_match = _CARTESIAN_RE.search(raw)
    if cartesian_match is not None:
        cartesians = _parse_csv_numbers(cartesian_match.group(1))
        if cartesians is not None and len(cartesians) >= 3:
            xyz_mm = (
                cartesians[0],
                cartesians[1],
                cartesians[2],
            )

    return ParsedStatus(
        state=state_match.group(1).strip(),
        raw=raw,
        joint_degrees=joint_degrees,
        xyz_mm=xyz_mm,
        complete=(
            raw.endswith(">")
            and joint_degrees is not None
            and xyz_mm is not None
        ),
    )
