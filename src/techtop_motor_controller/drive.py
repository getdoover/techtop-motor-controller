"""Modbus RTU driver for the Techtop TTA-3 series drive.

The TTA-3 is a rebadged Invertek Optidrive E3, so everything here follows the
Invertek *Optidrive E3 Fieldbus Guide* and *ODE-3 User Guide*. Register numbers
below are the documented (1-based) holding-register numbers; the address put on
the wire is ``register - 1``. The drive only has holding registers, and pydoover's
modbus interface reads those with ``register_type=4``.

The driver is pydoover-free on purpose: it only needs an object exposing the
``read_registers`` / ``write_registers`` coroutines of pydoover's
``ModbusInterface``, so it can be exercised against a fake in unit tests.

Drive state model (what the drive itself reports)
-------------------------------------------------
The E3 exposes its state as bits in status word 2 (register 2001):

    Ready    -- not tripped, mains present, hardware enable (DI1) closed
    Running  -- output stage enabled and driving the motor
    Tripped  -- a fault is latched; the trip code is in register 2016
    Standby  -- sleep function active (P-48)

and takes commands through the control word (register 1): bit 0 run, bit 1 fast
stop, bit 2 reset, bit 3 coast stop, with priority coast > fast stop > run. The
setpoint goes in register 2 in 0.1 Hz units (negative = reverse).

Two things the fieldbus guide is emphatic about, and this driver respects:

* Run/stop over Modbus only works with the drive in Modbus control mode
  (P-12 = 3 or 4) *and* the hardware enable on DI1 closed. Without P-12 = 3 the
  drive rejects writes to the control word outright.
* Once enabled, the drive expects a control-word telegram at least every P-36
  index 3 milliseconds (if that watchdog is enabled) or it trips / coasts. The
  application therefore rewrites the control word on every poll.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field

log = logging.getLogger(__name__)

HOLDING_REGISTERS = 4

# --- Control registers (registers 1-4, write) ------------------------------
REG_CONTROL_WORD = 1
REG_FREQUENCY_SETPOINT = 2  # S16, 0.1 Hz; capped at P-01
REG_RAMP_TIME = 4  # U16, 0.01 s; only used when P-12 = 4

CW_RUN = 1 << 0
CW_FAST_STOP = 1 << 1
CW_RESET = 1 << 2
CW_COAST_STOP = 1 << 3

# --- Status block (registers 2001-2016, one 16-register read) -------------
REG_STATUS_BLOCK = 2001
STATUS_BLOCK_LEN = 16
# Offsets within the block
_SW2 = 0  # status word 2
_OUT_FREQ = 1  # S16, 0.1 Hz
_OUT_CURRENT = 2  # U16, 0.1 A
_OUT_POWER = 3  # U16, 0.01 kW
_IO_WORD = 4  # digital input / output status
_TORQUE = 5  # U16, 4096 = 100 %
_DC_BUS = 6  # U16, volts
_HEATSINK_TEMP = 7  # S16, degC
_AI1 = 8  # S16, 0.1 %
_AI2 = 9  # U16, 0.1 %
_AO = 10  # U16, 0.1 %
_PI_OUT = 11  # U16, 0.1 %
_INTERNAL_TEMP = 12  # S16, degC
_OUT_VOLTAGE = 13  # U16, volts
_POT = 14  # IP66 pot
_TRIP_CODE = 15

# Status word 2 bits
SW2_READY = 1 << 0
SW2_RUNNING = 1 << 1
SW2_TRIPPED = 1 << 2
SW2_STANDBY = 1 << 3
SW2_FIRE_MODE = 1 << 4
SW2_AT_SPEED = 1 << 6
SW2_BELOW_MIN_SPEED = 1 << 7
SW2_OVERLOAD = 1 << 8
SW2_MAINS_LOSS = 1 << 9
SW2_HEATSINK_HOT = 1 << 10
SW2_BOARD_HOT = 1 << 11
SW2_SWITCHING_REDUCED = 1 << 12
SW2_REVERSE = 1 << 13

# IO status word bits (register 2005)
IO_DI1 = 1 << 0
IO_DI2 = 1 << 1
IO_DI3 = 1 << 2
IO_DI4 = 1 << 3
IO_DIGITAL_OUTPUT = 1 << 8
IO_RELAY_CLOSED = 1 << 9
IO_AI1_LOST = 1 << 12
IO_AI2_LOST = 1 << 13

# --- Meters (registers 32-35) ----------------------------------------------
REG_METER_BLOCK = 32
METER_BLOCK_LEN = 4  # kWh (0.1), MWh, run hours, run seconds-in-hour

# --- Direct parameter access -------------------------------------------------
# Every drive parameter P-xx is readable at holding register 128 + xx (verified
# on the bench: register 140 reads P-12). Values are in the drive's *internal*
# format, which differs per parameter, so only the ones decoded below are used.
PARAM_REGISTER_BASE = 128
P01_MAX_FREQUENCY = 1  # internal: 3000 = 50.0 Hz  (i.e. raw / 60)
P02_MIN_FREQUENCY = 2  # same scaling as P-01
P03_ACCEL_TIME = 3  # 0.01 s
P04_DECEL_TIME = 4  # 0.01 s
P07_MOTOR_VOLTAGE = 7  # V
P08_MOTOR_CURRENT = 8  # 0.1 A
P09_MOTOR_FREQUENCY = 9  # Hz
P12_CONTROL_SOURCE = 12
P31_KEYPAD_START_MODE = 31

INTERNAL_HZ_SCALE = 60.0  # internal speed units per Hz (3000 = 50 Hz)

CONTROL_SOURCES = {
    0: "terminal",
    1: "keypad",
    2: "keypad",
    3: "modbus",
    4: "modbus",
    5: "pi",
    6: "pi",
    7: "can",
    8: "can",
    9: "slave",
}
MODBUS_CONTROL_SOURCES = (3, 4)
P12_MODBUS_INTERNAL_RAMPS = 3

# ODE-3 User Guide section 10.1
TRIP_CODES = {
    0: "No fault",
    1: "Brake channel over current",
    2: "Brake resistor overload",
    3: "Output over current",
    4: "Motor thermal overload (I2t)",
    5: "Power stage trip",
    6: "Over voltage on DC bus",
    7: "Under voltage on DC bus",
    8: "Heatsink over temperature",
    9: "Under temperature",
    10: "Factory default parameters loaded",
    11: "External trip (DI3)",
    12: "Optibus comms loss",
    13: "DC bus ripple too high",
    14: "Input phase loss",
    15: "Output over current",
    16: "Faulty thermistor on heatsink",
    17: "Internal memory fault (IO)",
    18: "4-20mA signal lost",
    19: "Internal memory fault (DSP)",
    21: "Motor PTC thermistor trip",
    22: "Cooling fan fault",
    23: "Drive internal temperature too high",
    26: "Output fault",
    40: "Autotune fault",
    41: "Autotune fault: motor cable/connection",
    42: "Autotune fault: motor phases unbalanced",
    50: "Modbus comms loss",
    51: "CAN comms loss",
}


def trip_description(code: int) -> str:
    return TRIP_CODES.get(code, f"Trip code {code}")


def to_signed16(value: int) -> int:
    value &= 0xFFFF
    return value - 0x10000 if value & 0x8000 else value


def to_unsigned16(value: int) -> int:
    return value & 0xFFFF


def hz_to_setpoint_raw(frequency_hz: float) -> int:
    """Register 2 value for a frequency; sign carries direction."""
    return to_unsigned16(int(round(frequency_hz * 10)))


def setpoint_raw_to_hz(raw: int) -> float:
    return to_signed16(raw) / 10.0


def compose_control_word(
    run: bool = False,
    fast_stop: bool = False,
    coast_stop: bool = False,
    reset: bool = False,
) -> int:
    word = 0
    if run:
        word |= CW_RUN
    if fast_stop:
        word |= CW_FAST_STOP
    if coast_stop:
        word |= CW_COAST_STOP
    if reset:
        word |= CW_RESET
    return word


@dataclass
class DriveStatus:
    """One poll of the drive. ``contactable`` False means nothing else is valid."""

    contactable: bool = False

    # Drive state flags (status word 2)
    ready: bool = False
    running: bool = False
    tripped: bool = False
    standby: bool = False
    fire_mode: bool = False
    at_speed: bool = False
    below_min_speed: bool = False
    overload: bool = False
    mains_loss: bool = False
    heatsink_hot: bool = False
    board_hot: bool = False
    switching_reduced: bool = False
    reverse: bool = False
    status_word: int = 0

    trip_code: int = 0
    trip_description: str = ""

    # Operating values
    output_frequency_hz: float = 0.0
    motor_current_a: float = 0.0
    motor_power_kw: float = 0.0
    torque_pct: float = 0.0
    dc_bus_voltage_v: float = 0.0
    heatsink_temp_c: float = 0.0
    internal_temp_c: float = 0.0
    output_voltage_v: float = 0.0
    analog_input_1_pct: float = 0.0
    analog_input_2_pct: float = 0.0
    analog_output_pct: float = 0.0

    # IO
    io_word: int = 0
    di1: bool = False
    di2: bool = False
    di3: bool = False
    di4: bool = False
    digital_output_on: bool = False
    relay_closed: bool = False
    analog_input_1_lost: bool = False
    analog_input_2_lost: bool = False

    # Setpoint currently held by the drive (register 2)
    frequency_setpoint_hz: float | None = None

    # Meters (supplementary; None when the read failed)
    energy_kwh: float | None = None
    run_hours: float | None = None

    raw_status_block: list[int] = field(default_factory=list)

    @property
    def state_name(self) -> str:
        """The drive's own state as one word, in priority order."""
        if not self.contactable:
            return "disconnected"
        if self.tripped:
            return "tripped"
        if self.running:
            return "running"
        if self.standby:
            return "standby"
        if self.ready:
            return "ready"
        return "not_ready"

    @property
    def enable_present(self) -> bool:
        """The hardware enable on DI1 is closed."""
        return self.di1


@dataclass
class DriveParameters:
    """The handful of drive parameters the app cares about (direct reads)."""

    control_source_code: int | None = None
    max_frequency_hz: float | None = None
    min_frequency_hz: float | None = None
    accel_time_s: float | None = None
    decel_time_s: float | None = None
    motor_rated_voltage_v: float | None = None
    motor_rated_current_a: float | None = None
    motor_rated_frequency_hz: float | None = None
    keypad_start_mode: int | None = None

    @property
    def control_source(self) -> str | None:
        if self.control_source_code is None:
            return None
        return CONTROL_SOURCES.get(self.control_source_code, "unknown")

    @property
    def modbus_control(self) -> bool:
        return self.control_source_code in MODBUS_CONTROL_SOURCES


def decode_status_block(block: list[int]) -> DriveStatus:
    """Decode registers 2001-2016 into a :class:`DriveStatus`."""
    if block is None or len(block) < STATUS_BLOCK_LEN:
        return DriveStatus(contactable=False)

    sw = block[_SW2]
    io = block[_IO_WORD]
    status = DriveStatus(
        contactable=True,
        status_word=sw,
        ready=bool(sw & SW2_READY),
        running=bool(sw & SW2_RUNNING),
        tripped=bool(sw & SW2_TRIPPED),
        standby=bool(sw & SW2_STANDBY),
        fire_mode=bool(sw & SW2_FIRE_MODE),
        at_speed=bool(sw & SW2_AT_SPEED),
        below_min_speed=bool(sw & SW2_BELOW_MIN_SPEED),
        overload=bool(sw & SW2_OVERLOAD),
        mains_loss=bool(sw & SW2_MAINS_LOSS),
        heatsink_hot=bool(sw & SW2_HEATSINK_HOT),
        board_hot=bool(sw & SW2_BOARD_HOT),
        switching_reduced=bool(sw & SW2_SWITCHING_REDUCED),
        reverse=bool(sw & SW2_REVERSE),
        trip_code=block[_TRIP_CODE],
        output_frequency_hz=to_signed16(block[_OUT_FREQ]) / 10.0,
        motor_current_a=block[_OUT_CURRENT] / 10.0,
        motor_power_kw=block[_OUT_POWER] / 100.0,
        torque_pct=block[_TORQUE] / 40.96,
        dc_bus_voltage_v=float(block[_DC_BUS]),
        heatsink_temp_c=float(to_signed16(block[_HEATSINK_TEMP])),
        internal_temp_c=float(to_signed16(block[_INTERNAL_TEMP])),
        output_voltage_v=float(block[_OUT_VOLTAGE]),
        analog_input_1_pct=to_signed16(block[_AI1]) / 10.0,
        analog_input_2_pct=block[_AI2] / 10.0,
        analog_output_pct=block[_AO] / 10.0,
        io_word=io,
        di1=bool(io & IO_DI1),
        di2=bool(io & IO_DI2),
        di3=bool(io & IO_DI3),
        di4=bool(io & IO_DI4),
        digital_output_on=bool(io & IO_DIGITAL_OUTPUT),
        relay_closed=bool(io & IO_RELAY_CLOSED),
        analog_input_1_lost=bool(io & IO_AI1_LOST),
        analog_input_2_lost=bool(io & IO_AI2_LOST),
        raw_status_block=list(block),
    )
    if status.tripped:
        status.trip_description = trip_description(status.trip_code)
    return status


def decode_meters(block: list[int]) -> tuple[float | None, float | None]:
    """Registers 32-35 -> (energy_kwh, run_hours)."""
    if block is None or len(block) < METER_BLOCK_LEN:
        return None, None
    kwh, mwh, hours, seconds = block[:METER_BLOCK_LEN]
    return mwh * 1000.0 + kwh / 10.0, hours + seconds / 3600.0


def decode_parameters(raw: dict[int, int]) -> DriveParameters:
    """Decode a ``{parameter_number: raw}`` map into :class:`DriveParameters`."""
    params = DriveParameters()
    if P12_CONTROL_SOURCE in raw:
        params.control_source_code = raw[P12_CONTROL_SOURCE]
    if P01_MAX_FREQUENCY in raw:
        params.max_frequency_hz = raw[P01_MAX_FREQUENCY] / INTERNAL_HZ_SCALE
    if P02_MIN_FREQUENCY in raw:
        params.min_frequency_hz = raw[P02_MIN_FREQUENCY] / INTERNAL_HZ_SCALE
    if P03_ACCEL_TIME in raw:
        params.accel_time_s = raw[P03_ACCEL_TIME] / 100.0
    if P04_DECEL_TIME in raw:
        params.decel_time_s = raw[P04_DECEL_TIME] / 100.0
    if P07_MOTOR_VOLTAGE in raw:
        params.motor_rated_voltage_v = float(raw[P07_MOTOR_VOLTAGE])
    if P08_MOTOR_CURRENT in raw:
        params.motor_rated_current_a = raw[P08_MOTOR_CURRENT] / 10.0
    if P09_MOTOR_FREQUENCY in raw:
        params.motor_rated_frequency_hz = float(raw[P09_MOTOR_FREQUENCY])
    if P31_KEYPAD_START_MODE in raw:
        params.keypad_start_mode = raw[P31_KEYPAD_START_MODE]
    return params


class TechtopDrive:
    """Talks to one TTA-3 / Optidrive E3 through a pydoover ``ModbusInterface``.

    Parameters
    ----------
    modbus :
        pydoover ``ModbusInterface`` (``self.modbus_iface`` on the app), or any
        object with compatible ``read_registers`` / ``write_registers``.
    unit_id :
        Drive Modbus address (P-36 index 1, default 1).
    bus :
        Optional ``ModbusConfig`` element selecting the bus; ``None`` uses the
        one configured on the app.
    """

    def __init__(self, modbus, unit_id: int = 1, bus=None):
        self.modbus = modbus
        self.unit_id = unit_id
        self.bus = bus
        self.last_status: DriveStatus = DriveStatus()
        self.last_parameters: DriveParameters = DriveParameters()

    # -- low level -----------------------------------------------------------

    async def _read(self, register: int, count: int = 1, retries: int | None = None):
        kwargs = {} if retries is None else {"retries": retries}
        result = await self.modbus.read_registers(
            modbus_id=self.unit_id,
            start_address=register - 1,
            num_registers=count,
            register_type=HOLDING_REGISTERS,
            bus=self.bus,
            **kwargs,
        )
        if result is None:
            return None
        if isinstance(result, int):
            return [result]
        return list(result)

    async def _write_single(self, register: int, value: int) -> bool:
        """Write one register (Modbus FC06).

        The E3 answers FC06 for every writable register it has, while a
        multi-register FC16 to the control block is refused on the bench. So
        every write here is a single-register write.
        """
        return bool(
            await self.modbus.write_registers(
                modbus_id=self.unit_id,
                start_address=register - 1,
                values=[to_unsigned16(value)],
                register_type=HOLDING_REGISTERS,
                bus=self.bus,
            )
        )

    # -- reads ---------------------------------------------------------------

    async def read_status(self, include_meters: bool = False) -> DriveStatus:
        block = await self._read(REG_STATUS_BLOCK, STATUS_BLOCK_LEN)
        status = decode_status_block(block)
        if not status.contactable:
            self.last_status = status
            return status

        setpoint = await self._read(REG_FREQUENCY_SETPOINT, 1)
        if setpoint is not None:
            status.frequency_setpoint_hz = setpoint_raw_to_hz(setpoint[0])

        if include_meters:
            meters = await self._read(REG_METER_BLOCK, METER_BLOCK_LEN)
            status.energy_kwh, status.run_hours = decode_meters(meters)
        else:
            status.energy_kwh = self.last_status.energy_kwh
            status.run_hours = self.last_status.run_hours

        self.last_status = status
        return status

    async def read_parameter(self, number: int) -> int | None:
        result = await self._read(PARAM_REGISTER_BASE + number, 1)
        return None if result is None else result[0]

    async def read_parameters(self) -> DriveParameters:
        """Read P-01..P-12 and P-31 in two direct-register reads."""
        raw: dict[int, int] = {}
        first = await self._read(PARAM_REGISTER_BASE + 1, P12_CONTROL_SOURCE)
        if first is not None:
            for offset, value in enumerate(first):
                raw[offset + 1] = value
        p31 = await self.read_parameter(P31_KEYPAD_START_MODE)
        if p31 is not None:
            raw[P31_KEYPAD_START_MODE] = p31
        params = decode_parameters(raw)
        if first is not None:
            self.last_parameters = params
        return params

    # -- writes --------------------------------------------------------------

    async def write_setpoint(self, frequency_hz: float) -> bool:
        return await self._write_single(
            REG_FREQUENCY_SETPOINT, hz_to_setpoint_raw(frequency_hz)
        )

    async def write_control_word(self, word: int) -> bool:
        return await self._write_single(REG_CONTROL_WORD, word)

    async def write_command(self, control_word: int, frequency_hz: float) -> bool:
        """Refresh setpoint then control word -- the order matters for a start,
        so the drive never ramps to a stale reference."""
        ok = await self.write_setpoint(frequency_hz)
        return await self.write_control_word(control_word) and ok

    async def write_parameter(self, number: int, raw_value: int) -> bool:
        return await self._write_single(PARAM_REGISTER_BASE + number, raw_value)

    async def set_modbus_control(self) -> bool:
        """Put the drive into Modbus control mode (P-12 = 3)."""
        return await self.write_parameter(P12_CONTROL_SOURCE, P12_MODBUS_INTERNAL_RAMPS)
