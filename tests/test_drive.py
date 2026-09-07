"""Register decoding and command composition against values captured on the bench."""

import pytest

from techtop_motor_controller import drive
from techtop_motor_controller.drive import (
    CW_COAST_STOP,
    CW_FAST_STOP,
    CW_RESET,
    CW_RUN,
    DriveStatus,
    TechtopDrive,
    compose_control_word,
    decode_meters,
    decode_parameters,
    decode_status_block,
    hz_to_setpoint_raw,
    setpoint_raw_to_hz,
)

# Registers 2001-2016 read from the TTA-3-140022-3F12 on the bench with the
# hardware enable open: bit 15 of status word 2 toggles each second, DC bus
# 592 V, heatsink 30 C, internal 39 C, relay closed (drive healthy).
BENCH_NOT_READY = [0x8000, 0, 0, 0, 512, 0, 592, 30, 0, 0, 0, 0, 39, 0, 0, 0]
# Same drive with DI1 closed while still in terminal mode (P-12 = 0): DI1 is
# the run command there, so the drive reports ready + running + at-speed
# (0x43) with a 0 Hz reference. IO word carries DI1 + relay.
BENCH_ENABLED_TERMINAL_MODE = [0x0043, 0, 0, 0, 513, 0, 592, 30, 0, 0, 0, 0, 39, 0, 0, 0]
# Ready and idle, as the drive reports once P-12 = 3: ready + at-speed only.
BENCH_READY = [0x0041, 0, 0, 0, 513, 0, 592, 30, 0, 0, 0, 0, 39, 0, 0, 0]
# Registers 129-140 (P-01..P-12) from the bench: internal formats.
BENCH_PARAMS = [3000, 0, 500, 500, 0, 0, 400, 22, 50, 0, 30, 0]


def test_decode_not_ready_block():
    s = decode_status_block(BENCH_NOT_READY)
    assert s.contactable
    assert not s.ready and not s.running and not s.tripped
    assert s.state_name == "not_ready"
    assert s.dc_bus_voltage_v == 592
    assert s.heatsink_temp_c == 30
    assert s.internal_temp_c == 39
    assert s.relay_closed and not s.di1
    assert not s.enable_present


def test_decode_ready_block():
    s = decode_status_block(BENCH_READY)
    assert s.ready and s.at_speed and not s.running
    assert s.state_name == "ready"
    assert s.di1 and s.enable_present and s.relay_closed


def test_decode_enabled_in_terminal_mode_reads_as_running():
    s = decode_status_block(BENCH_ENABLED_TERMINAL_MODE)
    assert s.ready and s.running and s.at_speed
    assert s.state_name == "running"
    assert s.output_frequency_hz == 0.0


def test_decode_running_and_values():
    block = list(BENCH_READY)
    block[0] = 0x0043 | 0x0002  # running
    block[1] = 250  # 25.0 Hz
    block[2] = 18  # 1.8 A
    block[3] = 55  # 0.55 kW
    block[5] = 2048  # 50 % torque
    block[13] = 230
    s = decode_status_block(block)
    assert s.running and s.state_name == "running"
    assert s.output_frequency_hz == 25.0
    assert s.motor_current_a == 1.8
    assert s.motor_power_kw == 0.55
    assert round(s.torque_pct) == 50
    assert s.output_voltage_v == 230


def test_decode_trip():
    block = list(BENCH_READY)
    block[0] = 0x0004  # tripped
    block[15] = 7
    s = decode_status_block(block)
    assert s.tripped and s.state_name == "tripped"
    assert s.trip_code == 7
    assert "Under voltage" in s.trip_description


def test_decode_unknown_trip_code():
    block = list(BENCH_READY)
    block[0] = 0x0004
    block[15] = 99
    assert decode_status_block(block).trip_description == "Trip code 99"


def test_decode_negative_frequency_is_signed():
    block = list(BENCH_READY)
    block[1] = 0x10000 - 150  # -15.0 Hz
    assert decode_status_block(block).output_frequency_hz == -15.0


def test_decode_short_block_is_not_contactable():
    assert not decode_status_block(None).contactable
    assert not decode_status_block([1, 2, 3]).contactable
    assert DriveStatus().state_name == "disconnected"


def test_decode_parameters_from_bench():
    raw = {i + 1: v for i, v in enumerate(BENCH_PARAMS)}
    p = decode_parameters(raw)
    assert p.max_frequency_hz == 50.0
    assert p.min_frequency_hz == 0.0
    assert p.accel_time_s == 5.0 and p.decel_time_s == 5.0
    assert p.motor_rated_voltage_v == 400
    assert p.motor_rated_current_a == 2.2
    assert p.motor_rated_frequency_hz == 50
    assert p.control_source == "terminal" and not p.modbus_control


def test_modbus_control_sources():
    assert decode_parameters({12: 3}).modbus_control
    assert decode_parameters({12: 4}).modbus_control
    assert decode_parameters({12: 4}).control_source == "modbus"
    assert not decode_parameters({12: 1}).modbus_control
    assert decode_parameters({}).control_source is None


def test_decode_meters():
    energy, hours = decode_meters([123, 2, 10, 1800])
    assert energy == 2012.3
    assert hours == 10.5
    assert decode_meters(None) == (None, None)


def test_setpoint_scaling():
    assert hz_to_setpoint_raw(25.0) == 250
    assert hz_to_setpoint_raw(-25.0) == 0x10000 - 250
    assert setpoint_raw_to_hz(250) == 25.0
    assert setpoint_raw_to_hz(0x10000 - 250) == -25.0


def test_control_word_bits():
    assert compose_control_word() == 0
    assert compose_control_word(run=True) == CW_RUN == 1
    assert compose_control_word(fast_stop=True) == CW_FAST_STOP == 2
    assert compose_control_word(reset=True) == CW_RESET == 4
    assert compose_control_word(coast_stop=True) == CW_COAST_STOP == 8


class FakeModbus:
    """Records writes; answers reads from a register map keyed by address."""

    def __init__(self, registers=None):
        self.registers = dict(registers or {})
        self.writes = []
        self.fail_writes = False

    async def read_registers(self, modbus_id, start_address, num_registers, register_type, bus=None, **kw):
        assert register_type == 4
        values = []
        for a in range(start_address, start_address + num_registers):
            if a not in self.registers:
                return None
            values.append(self.registers[a])
        return values[0] if num_registers == 1 else values

    async def write_registers(self, modbus_id, start_address, values, register_type, bus=None, **kw):
        assert register_type == 4
        assert len(values) == 1, "the E3 only accepts single-register writes"
        if self.fail_writes:
            return False
        self.registers[start_address] = values[0]
        self.writes.append((start_address, values[0]))
        return True


def _bench_modbus():
    regs = {2000 + i: v for i, v in enumerate(BENCH_READY)}
    regs[1] = 0  # setpoint
    for i, v in enumerate(BENCH_PARAMS):
        regs[drive.PARAM_REGISTER_BASE + i] = v  # register 129+i -> address 128+i
    regs[drive.PARAM_REGISTER_BASE + drive.P31_KEYPAD_START_MODE - 1] = 1
    for i, v in enumerate([0, 0, 0, 0]):
        regs[31 + i] = v
    return FakeModbus(regs)


@pytest.mark.asyncio
async def test_drive_read_status_uses_documented_register_minus_one():
    fake = _bench_modbus()
    d = TechtopDrive(fake, unit_id=1)
    s = await d.read_status(include_meters=True)
    assert s.contactable and s.ready
    assert s.frequency_setpoint_hz == 0.0
    assert s.energy_kwh == 0.0 and s.run_hours == 0.0


@pytest.mark.asyncio
async def test_drive_read_parameters():
    d = TechtopDrive(_bench_modbus())
    p = await d.read_parameters()
    assert p.control_source == "terminal"
    assert p.max_frequency_hz == 50.0
    assert p.keypad_start_mode == 1


@pytest.mark.asyncio
async def test_drive_write_command_writes_setpoint_then_control_word():
    fake = _bench_modbus()
    d = TechtopDrive(fake)
    assert await d.write_command(CW_RUN, 30.0)
    # address 1 == register 2 (setpoint), address 0 == register 1 (control word)
    assert fake.writes == [(1, 300), (0, 1)]


@pytest.mark.asyncio
async def test_drive_write_command_reports_failure():
    fake = _bench_modbus()
    fake.fail_writes = True
    d = TechtopDrive(fake)
    assert not await d.write_command(CW_RUN, 30.0)


@pytest.mark.asyncio
async def test_drive_status_not_contactable_when_read_fails():
    d = TechtopDrive(FakeModbus({}))
    s = await d.read_status()
    assert not s.contactable
