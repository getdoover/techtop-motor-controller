"""Application-level checks that need no device agent: config plumbing,
frequency validation and the RPC status payload."""

import pytest
from pydoover.rpc import RPCError

from techtop_motor_controller.application import TechtopMotorControllerApplication
from techtop_motor_controller.drive import DriveParameters, DriveStatus, TechtopDrive

TEST_CONFIG = {
    "APP_KEY": "techtop_motor_controller_1",
    "APP_DISPLAY_NAME": "Test Drive",
    "modbus_config": {},
    "notifications": {},
    "modbus_unit_id": 1,
    "max_frequency_hz": 40.0,
    "min_frequency_hz": 5.0,
    "default_frequency_hz": 25.0,
}


def make_app():
    app = TechtopMotorControllerApplication(
        app_key="techtop_motor_controller_1", test_mode=True
    )
    app.config._inject_deployment_config(dict(TEST_CONFIG))
    return app


def test_config_defaults_apply():
    app = make_app()
    cfg = app.config
    assert cfg.control_enabled.value is True
    assert cfg.modbus_config.serial_baud.value == 115200
    assert cfg.stop_mode.value == "ramp"
    assert cfg.notifications.on_trip.value is True
    assert cfg.enable_pin is None
    assert cfg.clamp_frequency(100) == 40.0
    assert cfg.clamp_frequency(1) == 5.0


@pytest.mark.asyncio
async def test_frequency_validation_clamps_to_config_and_drive():
    app = make_app()
    await app.setup()
    assert app._validate_frequency(30) == 30.0
    assert app._validate_frequency("35.26") == 35.3
    assert app._validate_frequency(100) == 40.0
    assert app._validate_frequency(0) == 5.0
    app.params = DriveParameters(control_source_code=3, max_frequency_hz=35.0)
    assert app._validate_frequency(38) == 35.0
    with pytest.raises(RPCError):
        app._validate_frequency("fast")
    with pytest.raises(RPCError):
        app._validate_frequency(-1)
    with pytest.raises(RPCError):
        app._validate_direction("sideways")


@pytest.mark.asyncio
async def test_commands_refuse_without_comms_or_modbus_mode():
    app = make_app()
    await app.setup()
    with pytest.raises(RPCError) as e:
        await app.command_start(source="test")
    assert e.value.code == "NOT_CONNECTED"

    payload = app.status_dict()
    assert payload["controller_state"] == "disconnected"
    assert payload["requested_frequency_hz"] == 25.0
    assert payload["control_enabled"] is True
    assert payload["modbus_control"] is False


class _ParamModbus:
    """Minimal modbus stand-in for nameplate writes."""

    def __init__(self, refuse=(), raise_on_write=False):
        self.single_writes = []
        self.refuse = set(refuse)
        self.raise_on_write = raise_on_write

    async def write_single_register(self, modbus_id, address, value, **kw):
        if self.raise_on_write:
            raise RuntimeError("writeSingleRegister UNIMPLEMENTED")
        self.single_writes.append((address, value))
        return address not in self.refuse


BENCH_NAMEPLATE = DriveParameters(
    control_source_code=3,
    motor_rated_voltage_v=400.0,
    motor_rated_current_a=2.2,
    motor_rated_frequency_hz=50.0,
    motor_rated_speed_rpm=0.0,
)


async def _nameplate_app(extra_config, modbus, running=False):
    app = TechtopMotorControllerApplication(
        app_key="techtop_motor_controller_1", test_mode=True
    )
    app.config._inject_deployment_config({**TEST_CONFIG, **extra_config})
    await app.setup()
    app.drive = TechtopDrive(modbus)
    app.drive.last_status = DriveStatus(contactable=True, running=running)
    return app


def test_nameplate_defaults_to_nothing():
    assert make_app().config.nameplate == {}


@pytest.mark.asyncio
async def test_nameplate_writes_only_differing_values_with_fc06():
    modbus = _ParamModbus()
    app = await _nameplate_app(
        {
            "motor_rated_current_a": 2.1,
            "motor_rated_voltage_v": 400,
            "motor_rated_speed_rpm": 1420,
        },
        modbus,
    )
    assert await app._apply_nameplate(BENCH_NAMEPLATE) is True
    # P-08 -> address 135 (2.1 A = 21), P-10 -> address 137; P-07 already 400
    assert modbus.single_writes == [(135, 21), (137, 1420)]


@pytest.mark.asyncio
async def test_nameplate_not_written_while_running_or_monitor_only():
    modbus = _ParamModbus()
    app = await _nameplate_app({"motor_rated_current_a": 2.1}, modbus, running=True)
    assert await app._apply_nameplate(BENCH_NAMEPLATE) is False
    app = await _nameplate_app(
        {"motor_rated_current_a": 2.1, "control_enabled": False}, modbus
    )
    assert await app._apply_nameplate(BENCH_NAMEPLATE) is False
    assert modbus.single_writes == []


@pytest.mark.asyncio
async def test_refused_nameplate_value_is_not_retried():
    modbus = _ParamModbus(refuse={135})
    app = await _nameplate_app({"motor_rated_current_a": 2.3}, modbus)
    assert await app._apply_nameplate(BENCH_NAMEPLATE) is False
    assert await app._apply_nameplate(BENCH_NAMEPLATE) is False
    assert modbus.single_writes == [(135, 23)]


@pytest.mark.asyncio
async def test_nameplate_write_error_does_not_raise():
    modbus = _ParamModbus(raise_on_write=True)
    app = await _nameplate_app({"motor_rated_current_a": 2.1}, modbus)
    assert await app._apply_nameplate(BENCH_NAMEPLATE) is False
