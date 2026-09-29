"""The commissioning parameter table: scaling, ids and the allowlist."""

import pytest

from techtop_motor_controller.drive import TechtopDrive
from techtop_motor_controller.parameters import (
    FORBIDDEN_PARAMETERS,
    PARAMETERS,
    PARAMETERS_BY_NUMBER,
    decode_p36,
    decode_value,
    format_parameter_id,
    is_step_multiple,
    parse_parameter_id,
)

# (parameter, engineering value, raw register value); the raw values for
# P-01..P-10 are the bench drive's (see test_drive.BENCH_PARAMS).
SCALING_CASES = [
    (1, 50.0, 3000),
    (1, 60.5, 3630),
    (2, 0.0, 0),
    (2, 12.5, 750),
    (3, 5.0, 500),
    (3, 0.25, 25),
    (4, 5.0, 500),
    (4, 600.0, 60000),
    (5, 2, 2),
    (7, 400, 400),
    (8, 2.2, 22),
    (8, 1.0, 10),
    (9, 50, 50),
    (10, 1390, 1390),
    (24, 1.5, 150),
    (12, 3, 3),
]


@pytest.mark.parametrize(("number", "value", "raw"), SCALING_CASES)
def test_scaling_round_trips(number, value, raw):
    spec = PARAMETERS_BY_NUMBER[number]
    assert spec.to_raw(value) == raw
    assert spec.from_raw(raw) == value
    assert spec.from_raw(spec.to_raw(value)) == value


def test_every_step_round_trips_across_the_range():
    for spec in PARAMETERS:
        low = spec.minimum or 0
        for k in range(50):
            value = round(low + k * spec.step, spec.decimals)
            assert spec.from_raw(spec.to_raw(value)) == value, spec.id


def test_whole_number_parameters_decode_as_int():
    assert isinstance(PARAMETERS_BY_NUMBER[10].from_raw(1390), int)
    assert isinstance(PARAMETERS_BY_NUMBER[8].from_raw(22), float)


def test_parameter_ids():
    assert format_parameter_id(9) == "P-09"
    assert parse_parameter_id("P-09") == 9
    assert parse_parameter_id("p9") == 9
    assert parse_parameter_id("P-36") == 36
    assert parse_parameter_id("9") is None
    assert parse_parameter_id(9) is None
    assert parse_parameter_id("P-0x") is None
    assert [s.id for s in PARAMETERS][:3] == ["P-01", "P-02", "P-03"]


def test_allowlist_minimum_is_writable():
    for number in (1, 2, 3, 4, 7, 8, 9, 10):
        assert PARAMETERS_BY_NUMBER[number].writable


def test_forbidden_parameters_are_never_writable():
    for number in (12, 14, 36, 37, 38):
        assert number in FORBIDDEN_PARAMETERS
        spec = PARAMETERS_BY_NUMBER.get(number)
        assert spec is None or not spec.writable


def test_motor_data_needs_the_motor_stopped():
    for number in (5, 7, 8, 9, 10):
        assert PARAMETERS_BY_NUMBER[number].stop_required
    for number in (1, 2, 3, 4, 24):
        assert not PARAMETERS_BY_NUMBER[number].stop_required


def test_p36_decodes_the_packed_word():
    assert decode_p36(0x4601) == (1, 115.2, "t 3000")
    assert decode_p36(0x0205) == (5, 9.6, "disabled")
    assert decode_value(PARAMETERS_BY_NUMBER[36], 0x4601) == 1
    assert decode_value(PARAMETERS_BY_NUMBER[1], None) is None


def test_step_multiples():
    assert is_step_multiple(50.3, 0.1)
    assert is_step_multiple(5.25, 0.01)
    assert not is_step_multiple(2.5, 1)
    assert not is_step_multiple(2.25, 0.1)


class _Registers:
    def __init__(self, registers):
        self.registers = registers
        self.reads = []

    async def read_registers(self, modbus_id, start_address, num_registers, **kw):
        self.reads.append((start_address, num_registers))
        values = [
            self.registers.get(a)
            for a in range(start_address, start_address + num_registers)
        ]
        if None in values:
            return None
        return values[0] if num_registers == 1 else values


@pytest.mark.asyncio
async def test_read_parameter_values_groups_contiguous_runs():
    # register 128 + P-xx is wire address 127 + P-xx
    regs = {127 + n: n * 10 for n in range(1, 13)}
    regs[127 + 24] = 150
    fake = _Registers(regs)
    raw = await TechtopDrive(fake).read_parameter_values([1, 2, 3, 12, 24, 36])
    # P-01..P-03, P-12, P-24 and P-36 as four reads; P-36 did not answer
    assert fake.reads == [(128, 3), (139, 1), (151, 1), (163, 1)]
    assert raw == {1: 10, 2: 20, 3: 30, 12: 120, 24: 150}
