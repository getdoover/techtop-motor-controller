"""Drive parameters exposed to the commissioning panel.

Each entry says how to turn the drive's internal register value (register
128 + P-xx) into engineering units and back, the documented range, and whether
the panel may write it. Ranges and units follow the Optidrive E3 IP20 User
Guide (V1.05, section 6.1). Internal scalings are the ones verified on the
bench (see ``drive.py``); P-24 shares P-03/P-04's 0.01 s format.

Stop-only flags are this app's own rule. The user guide lists none per
parameter; the motor data (P-07..P-10) and the stopping mode (P-05) are only
changed with the motor stopped because changing them mid-run changes how the
drive is controlling the motor it is turning. P-09 also resets P-10 and the
preset speeds on the drive.

Never writable from the panel, whatever the config: P-12 (control source; a
wrong value takes the drive out of Modbus control or starts it from the
terminals), P-36 (address, baud, comms-loss watchdog: keypad only, and a wrong
value cuts this app off), P-14 / P-37 / P-38 (keypad access codes and lock).
The E3 has no factory-reset parameter; that is a keypad key combination.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

from .drive import (
    INTERNAL_HZ_SCALE,
    P01_MAX_FREQUENCY,
    P02_MIN_FREQUENCY,
    P03_ACCEL_TIME,
    P04_DECEL_TIME,
    P07_MOTOR_VOLTAGE,
    P08_MOTOR_CURRENT,
    P09_MOTOR_FREQUENCY,
    P10_MOTOR_SPEED,
    P12_CONTROL_SOURCE,
)

P05_STOPPING_MODE = 5
P24_FAST_STOP_RAMP = 24
P36_SERIAL_COMMS = 36

# Never written from the panel. P-12 and P-36 are listed read-only; the access
# codes are not listed at all.
FORBIDDEN_PARAMETERS = {
    P12_CONTROL_SOURCE: "The control source (P-12) is set on the drive keypad only",
    14: "The extended menu access code (P-14) is set on the drive keypad only",
    P36_SERIAL_COMMS: "The Modbus address, baud rate and comms-loss timeout (P-36) are set on the drive keypad only",
    37: "The access code definition (P-37) is set on the drive keypad only",
    38: "The parameter lock (P-38) is set on the drive keypad only",
}

P36_BAUD_KBPS = {2: 9.6, 3: 19.2, 4: 38.4, 5: 57.6}
P36_TRIP_SETTINGS = {
    0: "disabled",
    1: "t 30",
    2: "t 300",
    3: "t 1000",
    4: "t 3000",
    5: "r 30",
    6: "r 300",
    7: "r 1000",
    8: "r 3000",
}


@dataclass(frozen=True)
class ParameterSpec:
    number: int
    name: str
    units: str
    scale: float  # raw register units per engineering unit
    minimum: float | None
    maximum: float | None
    step: float
    stop_required: bool
    description: str
    writable: bool = True

    @property
    def id(self) -> str:
        return format_parameter_id(self.number)

    @property
    def decimals(self) -> int:
        return max(0, -math.floor(math.log10(self.step))) if self.step < 1 else 0

    def to_raw(self, value: float) -> int:
        return round(float(value) * self.scale)

    def from_raw(self, raw: int) -> float | int:
        value = raw / self.scale
        if self.decimals == 0:
            return round(value)
        return round(value, self.decimals)


PARAMETERS: tuple[ParameterSpec, ...] = (
    ParameterSpec(
        P01_MAX_FREQUENCY,
        "Maximum frequency",
        "Hz",
        INTERNAL_HZ_SCALE,
        None,  # P-02
        500.0,
        0.1,
        False,
        "Highest output frequency. Setpoints above it are refused by the drive. Minimum is P-02.",
    ),
    ParameterSpec(
        P02_MIN_FREQUENCY,
        "Minimum frequency",
        "Hz",
        INTERNAL_HZ_SCALE,
        0.0,
        None,  # P-01
        0.1,
        False,
        "Lowest output frequency while running. Maximum is P-01.",
    ),
    ParameterSpec(
        P03_ACCEL_TIME,
        "Acceleration time",
        "s",
        100,
        0.0,
        600.0,
        0.01,
        False,
        "Ramp time from 0 Hz to the motor rated frequency (P-09).",
    ),
    ParameterSpec(
        P04_DECEL_TIME,
        "Deceleration time",
        "s",
        100,
        0.0,
        600.0,
        0.01,
        False,
        "Ramp time from the motor rated frequency (P-09) to standstill. 0 uses P-24.",
    ),
    ParameterSpec(
        P05_STOPPING_MODE,
        "Stopping mode",
        "",
        1,
        0,
        4,
        1,
        True,
        "0 ramp to stop (ride through mains loss), 1 coast, 2 ramp (fast stop on mains loss), "
        "3 ramp with AC flux braking, 4 ramp (no mains loss action).",
    ),
    ParameterSpec(
        P07_MOTOR_VOLTAGE,
        "Motor rated voltage",
        "V",
        1,
        0,
        500,
        1,
        True,
        "Motor nameplate voltage.",
    ),
    ParameterSpec(
        P08_MOTOR_CURRENT,
        "Motor rated current",
        "A",
        10,
        0.0,
        None,  # the drive's own rating; the drive refuses more
        0.1,
        True,
        "Motor nameplate current. The drive refuses a value above its own rated current.",
    ),
    ParameterSpec(
        P09_MOTOR_FREQUENCY,
        "Motor rated frequency",
        "Hz",
        1,
        10,
        500,
        1,
        True,
        "Motor nameplate frequency. Changing it resets P-10 and the preset speeds on the drive.",
    ),
    ParameterSpec(
        P10_MOTOR_SPEED,
        "Motor rated speed",
        "rpm",
        1,
        0,
        30000,
        1,
        True,
        "Motor nameplate speed. 0 shows speed in Hz and disables slip compensation.",
    ),
    ParameterSpec(
        P12_CONTROL_SOURCE,
        "Control source",
        "",
        1,
        0,
        9,
        1,
        True,
        "0 terminal, 1-2 keypad, 3-4 Modbus, 5-6 PI, 7-8 CAN, 9 slave. Set on the drive keypad only.",
        writable=False,
    ),
    ParameterSpec(
        P24_FAST_STOP_RAMP,
        "Fast stop ramp time",
        "s",
        100,
        0.0,
        600.0,
        0.01,
        False,
        "Ramp used by a fast stop and on mains loss (P-05 = 2 or 3). 0 coasts to stop.",
    ),
    ParameterSpec(
        P36_SERIAL_COMMS,
        "Modbus address",
        "",
        1,
        0,
        63,
        1,
        True,
        "Drive Modbus address (P-36 index 1). P-36 is set on the drive keypad only.",
        writable=False,
    ),
)

PARAMETERS_BY_NUMBER = {spec.number: spec for spec in PARAMETERS}

_ID_PATTERN = re.compile(r"^\s*P-?0*(\d{1,3})\s*$", re.IGNORECASE)


def format_parameter_id(number: int) -> str:
    return f"P-{number:02d}"


def parse_parameter_id(value) -> int | None:
    """``"P-09"`` (or ``"p9"``) -> 9; anything else -> None."""
    if not isinstance(value, str):
        return None
    match = _ID_PATTERN.match(value)
    return int(match.group(1)) if match else None


def decode_p36(raw: int) -> tuple[int, float, str]:
    """Packed P-36 word -> (address, baud kbps, comms-loss setting)."""
    address = raw & 0xFF
    baud = P36_BAUD_KBPS.get((raw >> 8) & 0x0F, 115.2)
    trip = P36_TRIP_SETTINGS.get((raw >> 12) & 0x0F, f"setting {(raw >> 12) & 0x0F}")
    return address, baud, trip


def decode_value(spec: ParameterSpec, raw: int | None) -> float | int | None:
    if raw is None:
        return None
    if spec.number == P36_SERIAL_COMMS:
        return decode_p36(raw)[0]
    return spec.from_raw(raw)


def is_step_multiple(value: float, step: float) -> bool:
    ratio = value / step
    return abs(ratio - round(ratio)) < 1e-6
