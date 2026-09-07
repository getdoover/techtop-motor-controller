"""Application-level checks that need no device agent: config plumbing,
frequency validation and the RPC status payload."""

import pytest
from pydoover.rpc import RPCError

from techtop_motor_controller.application import TechtopMotorControllerApplication
from techtop_motor_controller.drive import DriveParameters

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
