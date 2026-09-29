"""Commissioning RPCs: get_diagnostics, read_parameters and write_parameter."""

import pytest
from pydoover.rpc import RPCError

from techtop_motor_controller.application import (
    CONFIG_MANAGED_DESCRIPTION,
    TechtopMotorControllerApplication,
)
from techtop_motor_controller.drive import TechtopDrive

from .test_application import TEST_CONFIG
from .test_drive import BENCH_PARAMS, BENCH_READY

DIAGNOSTIC_KEYS = {
    "output_hz",
    "output_current_a",
    "motor_rpm",
    "dc_bus_v",
    "heatsink_c",
    "drive_state",
    "trip_code",
    "trip_description",
    "run_hours",
    "recent_trips",
    "comms_ok",
}
PARAMETER_KEYS = {
    "id",
    "name",
    "value",
    "units",
    "min",
    "max",
    "step",
    "writable",
    "stop_required",
    "description",
}


def param_address(number: int) -> int:
    """Wire address of parameter P-xx (register 128 + xx, minus one)."""
    return 127 + number


class DriveModbus:
    """A bench-like drive: status block, setpoint, meters and parameters."""

    def __init__(self):
        self.registers = {2000 + i: v for i, v in enumerate(BENCH_READY)}
        self.registers[1] = 0
        for i, v in enumerate(BENCH_PARAMS):
            self.registers[param_address(i + 1)] = v
        self.registers[param_address(24)] = 0
        self.registers[param_address(36)] = 0x4601
        for i, v in enumerate([123, 2, 10, 1800]):
            self.registers[31 + i] = v
        self.single_writes = []
        self.reads = []
        self.offline = False
        self.refuse_writes = False
        self.raise_on_write = False
        self.store_offset = 0  # the drive stores a different value (clamps)

    def set_running(self, running=True):
        self.registers[2000] = 0x0043 if running else 0x0041

    async def read_registers(self, modbus_id, start_address, num_registers, **kw):
        self.reads.append((start_address, num_registers))
        if self.offline:
            return None
        values = []
        for a in range(start_address, start_address + num_registers):
            if a not in self.registers:
                return None
            values.append(self.registers[a])
        return values[0] if num_registers == 1 else values

    async def write_single_register(self, modbus_id, address, value, **kw):
        if self.raise_on_write:
            raise RuntimeError("writeSingleRegister UNIMPLEMENTED")
        if self.offline or self.refuse_writes:
            return False
        self.single_writes.append((address, value))
        self.registers[address] = value + self.store_offset
        return True


class Ctx:
    def __init__(self):
        self.actor = {
            "id": "u1",
            "name": "Commissioning Tech",
            "email": "tech@example.com",
        }


async def make_app(extra_config=None, modbus=None):
    app = TechtopMotorControllerApplication(
        app_key="techtop_motor_controller_1", test_mode=True
    )
    app.config._inject_deployment_config({**TEST_CONFIG, **(extra_config or {})})
    await app.setup()
    modbus = modbus or DriveModbus()
    app.drive = TechtopDrive(modbus)
    return app, modbus


async def write(app, parameter, value):
    return await app.rpc_write_parameter(
        Ctx(), {"parameter": parameter, "value": value}
    )


async def error_code(app, parameter, value) -> str:
    with pytest.raises(RPCError) as e:
        await write(app, parameter, value)
    assert e.value.message
    return e.value.code


# --- get_diagnostics ---------------------------------------------------------


@pytest.mark.asyncio
async def test_diagnostics_payload_shape():
    app, modbus = await make_app()
    diag = await app.rpc_get_diagnostics(Ctx(), {})
    assert set(diag) == DIAGNOSTIC_KEYS
    assert diag["comms_ok"] is True
    assert diag["drive_state"] == "ready"
    assert diag["output_hz"] == 0.0
    assert diag["output_current_a"] == 0.0
    assert diag["dc_bus_v"] == 592.0
    assert diag["heatsink_c"] == 30.0
    assert diag["run_hours"] == 10.5
    assert diag["trip_code"] is None and diag["trip_description"] is None
    # Not in the E3's Modbus map
    assert diag["recent_trips"] is None
    # P-10 unknown / 0: no rpm estimate
    assert diag["motor_rpm"] is None
    # One read cycle: status block, setpoint, meters
    assert modbus.reads == [(2000, 16), (1, 1), (31, 4)]


@pytest.mark.asyncio
async def test_diagnostics_trip_and_rpm_estimate():
    app, modbus = await make_app()
    modbus.registers[2000] = 0x0004
    modbus.registers[2015] = 7
    modbus.registers[2001] = 250  # 25.0 Hz out
    app.params.motor_rated_frequency_hz = 50.0
    app.params.motor_rated_speed_rpm = 1390.0
    diag = await app.rpc_get_diagnostics(Ctx(), {})
    assert diag["drive_state"] == "tripped"
    assert diag["trip_code"] == 7
    assert "Under voltage" in diag["trip_description"]
    assert diag["motor_rpm"] == 695


@pytest.mark.asyncio
async def test_diagnostics_without_comms_is_all_null():
    app, modbus = await make_app()
    modbus.offline = True
    diag = await app.rpc_get_diagnostics(Ctx(), {})
    assert set(diag) == DIAGNOSTIC_KEYS
    assert diag["comms_ok"] is False
    assert diag["drive_state"] == "disconnected"
    for key in DIAGNOSTIC_KEYS - {"comms_ok", "drive_state"}:
        assert diag[key] is None, key


@pytest.mark.asyncio
async def test_get_status_is_unchanged():
    app, _ = await make_app()
    assert set(await app.rpc_get_status(Ctx(), {})) == set(app.status_dict())
    assert "comms_active" in app.status_dict()


# --- read_parameters ---------------------------------------------------------


@pytest.mark.asyncio
async def test_read_parameters_payload():
    app, _ = await make_app()
    result = await app.rpc_read_parameters(Ctx(), {})
    params = {p["id"]: p for p in result["parameters"]}
    for p in params.values():
        assert set(p) == PARAMETER_KEYS
    assert params["P-01"]["value"] == 50.0 and params["P-01"]["units"] == "Hz"
    assert params["P-01"]["min"] == 0.0  # P-02
    assert params["P-02"]["max"] == 50.0  # P-01
    assert params["P-03"]["value"] == 5.0 and params["P-03"]["step"] == 0.01
    assert params["P-08"]["value"] == 2.2 and params["P-08"]["max"] is None
    assert params["P-09"] == {**params["P-09"], "value": 50, "stop_required": True}
    assert params["P-12"]["writable"] is False and params["P-12"]["value"] == 0
    assert params["P-36"]["writable"] is False and params["P-36"]["value"] == 1
    assert "115.2 kbps" in params["P-36"]["description"]
    assert "t 3000" in params["P-36"]["description"]
    for pid in ("P-01", "P-02", "P-03", "P-04", "P-07", "P-08", "P-09", "P-10"):
        assert params[pid]["writable"] is True, pid
    for pid in ("P-14", "P-37", "P-38"):
        assert pid not in params


@pytest.mark.asyncio
async def test_read_parameters_without_comms_reports_null_values():
    app, modbus = await make_app()
    modbus.offline = True
    result = await app.rpc_read_parameters(Ctx(), {})
    assert result["parameters"]
    assert all(p["value"] is None for p in result["parameters"])


# --- write_parameter: happy path and scaling ---------------------------------


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("parameter", "value", "raw"),
    [
        ("P-01", 45.5, 2730),
        ("P-02", 10.0, 600),
        ("P-03", 2.5, 250),
        ("P-04", 12.75, 1275),
        ("P-05", 1, 1),
        ("P-07", 415, 415),
        ("P-08", 1.0, 10),
        ("P-09", 60, 60),
        ("P-10", 1390, 1390),
        ("P-24", 0.5, 50),
    ],
)
async def test_write_scales_writes_fc06_and_reads_back(parameter, value, raw):
    app, modbus = await make_app()
    result = await write(app, parameter, value)
    assert result == {"parameter": parameter, "value": value}
    number = int(parameter[2:])
    assert modbus.single_writes == [(param_address(number), raw)]


@pytest.mark.asyncio
async def test_write_logs_value_and_actor(caplog):
    app, _ = await make_app()
    with caplog.at_level("INFO"):
        await write(app, "P-08", 1.5)
    assert "P-08 set to 1.5 A" in caplog.text
    assert "Commissioning Tech" in caplog.text
    assert "was 2.2" in caplog.text


@pytest.mark.asyncio
async def test_write_forces_a_parameter_refresh():
    app, _ = await make_app()
    app._params_read_at = 123.0
    await write(app, "P-01", 45)
    assert app._params_read_at is None


# --- allowlist ----------------------------------------------------------------


@pytest.mark.asyncio
@pytest.mark.parametrize("parameter", ["P-12", "P-14", "P-36", "P-37", "P-38"])
async def test_forbidden_parameters_are_refused(parameter):
    app, modbus = await make_app()
    assert await error_code(app, parameter, 3) == "NOT_ALLOWED"
    assert modbus.single_writes == []


@pytest.mark.asyncio
@pytest.mark.parametrize("parameter", ["P-06", "P-15", "P-99", "9", None, "bogus"])
async def test_parameters_outside_the_allowlist_are_refused(parameter):
    app, modbus = await make_app()
    assert await error_code(app, parameter, 1) == "NOT_ALLOWED"
    assert modbus.single_writes == []


@pytest.mark.asyncio
async def test_monitor_only_refuses_writes_and_reports_read_only():
    app, modbus = await make_app({"control_enabled": False})
    assert await error_code(app, "P-03", 2.0) == "NOT_ALLOWED"
    params = (await app.rpc_read_parameters(Ctx(), {}))["parameters"]
    assert not any(p["writable"] for p in params)
    assert modbus.single_writes == []


# --- range --------------------------------------------------------------------


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("parameter", "value"),
    [
        ("P-03", 600.01),
        ("P-03", -1),
        ("P-09", 5),
        ("P-10", 30001),
        ("P-05", 5),
        ("P-05", 2.5),  # not a whole number
        ("P-08", 1.05),  # finer than 0.1 A
        ("P-02", 55.0),  # above P-01 (50 Hz)
        ("P-03", "fast"),
        ("P-03", True),
        ("P-03", None),
        ("P-03", float("nan")),
    ],
)
async def test_out_of_range_values_are_refused(parameter, value):
    app, modbus = await make_app()
    assert await error_code(app, parameter, value) == "OUT_OF_RANGE"
    assert modbus.single_writes == []


@pytest.mark.asyncio
async def test_value_the_drive_refuses_is_out_of_range():
    app, modbus = await make_app()
    modbus.refuse_writes = True  # e.g. P-08 above the drive's own rating
    assert await error_code(app, "P-08", 9.9) == "OUT_OF_RANGE"


# --- running, readback, comms -------------------------------------------------


@pytest.mark.asyncio
@pytest.mark.parametrize("parameter", ["P-05", "P-07", "P-08", "P-09", "P-10"])
async def test_stop_only_parameters_refused_while_running(parameter):
    app, modbus = await make_app()
    modbus.set_running()
    assert await error_code(app, parameter, 1) == "DRIVE_RUNNING"
    assert modbus.single_writes == []


@pytest.mark.asyncio
async def test_ramps_can_change_while_running():
    app, modbus = await make_app()
    modbus.set_running()
    assert (await write(app, "P-03", 3.0))["value"] == 3.0


@pytest.mark.asyncio
async def test_readback_mismatch():
    app, modbus = await make_app()
    modbus.store_offset = 1
    with pytest.raises(RPCError) as e:
        await write(app, "P-07", 380)
    assert e.value.code == "READBACK_MISMATCH"
    assert "381" in e.value.message


@pytest.mark.asyncio
async def test_comms_error_when_the_drive_does_not_answer():
    app, modbus = await make_app()
    modbus.offline = True
    assert await error_code(app, "P-03", 2.0) == "COMMS_ERROR"


@pytest.mark.asyncio
async def test_comms_error_when_the_write_raises():
    app, modbus = await make_app()
    modbus.raise_on_write = True
    assert await error_code(app, "P-03", 2.0) == "COMMS_ERROR"


@pytest.mark.asyncio
async def test_comms_error_when_the_link_drops_during_the_write():
    app, modbus = await make_app()
    original = modbus.write_single_register

    async def drop(*args, **kwargs):
        modbus.offline = True
        return await original(*args, **kwargs)

    modbus.write_single_register = drop
    assert await error_code(app, "P-03", 2.0) == "COMMS_ERROR"


# --- nameplate config interaction ------------------------------------------------

NAMEPLATE_CONFIG = {
    "motor_rated_voltage_v": 400,
    "motor_rated_current_a": 1.0,
    "motor_rated_frequency_hz": 50,
    "motor_rated_speed_rpm": 1390,
}


@pytest.mark.asyncio
async def test_config_managed_nameplate_is_read_only():
    app, modbus = await make_app(NAMEPLATE_CONFIG)
    params = {
        p["id"]: p for p in (await app.rpc_read_parameters(Ctx(), {}))["parameters"]
    }
    for pid in ("P-07", "P-08", "P-09", "P-10"):
        assert params[pid]["writable"] is False
        assert params[pid]["description"] == CONFIG_MANAGED_DESCRIPTION
        with pytest.raises(RPCError) as e:
            await write(app, pid, 1)
        assert e.value.code == "NOT_ALLOWED"
        assert CONFIG_MANAGED_DESCRIPTION in e.value.message
    # Other parameters stay writable
    assert params["P-03"]["writable"] is True
    assert modbus.single_writes == []


@pytest.mark.asyncio
async def test_only_the_configured_nameplate_fields_are_managed():
    app, _ = await make_app({"motor_rated_current_a": 1.0})
    params = {
        p["id"]: p for p in (await app.rpc_read_parameters(Ctx(), {}))["parameters"]
    }
    assert params["P-08"]["writable"] is False
    assert params["P-07"]["writable"] is True
    assert (await write(app, "P-07", 380))["value"] == 380
    assert await error_code(app, "P-08", 1.5) == "NOT_ALLOWED"


@pytest.mark.asyncio
async def test_nameplate_writable_when_config_unset():
    app, modbus = await make_app()
    assert app.config.nameplate == {}
    params = {
        p["id"]: p for p in (await app.rpc_read_parameters(Ctx(), {}))["parameters"]
    }
    for pid, value in (("P-07", 400), ("P-08", 1.0), ("P-09", 50), ("P-10", 1390)):
        assert params[pid]["writable"] is True
        assert params[pid]["description"] != CONFIG_MANAGED_DESCRIPTION
        assert (await write(app, pid, value))["value"] == value
    assert len(modbus.single_writes) == 4
