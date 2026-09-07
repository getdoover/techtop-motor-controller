"""Smoke tests: modules import, schema exports are well formed."""

import json

from pydoover.config import Schema
from pydoover.tags import Tags
from pydoover.ui import UI


def test_import_app():
    from techtop_motor_controller.application import TechtopMotorControllerApplication

    assert TechtopMotorControllerApplication.config_cls is not None
    assert TechtopMotorControllerApplication.tags_cls is not None
    assert TechtopMotorControllerApplication.ui_cls is not None


def test_config_schema():
    from techtop_motor_controller.app_config import TechtopMotorControllerConfig

    assert issubclass(TechtopMotorControllerConfig, Schema)
    schema = TechtopMotorControllerConfig.to_schema()
    props = schema["properties"]
    assert "modbus_config" in props
    assert props["modbus_config"]["properties"]["serial_baud"]["default"] == 115200
    assert props["modbus_unit_id"]["default"] == 1
    assert props["control_enabled"]["default"] is True
    assert "notifications" in props


def test_tags():
    from techtop_motor_controller.app_tags import TechtopMotorControllerTags

    assert issubclass(TechtopMotorControllerTags, Tags)


def test_ui():
    from techtop_motor_controller.app_ui import TechtopMotorControllerUI

    assert issubclass(TechtopMotorControllerUI, UI)


def test_config_export(tmp_path):
    from techtop_motor_controller.app_config import TechtopMotorControllerConfig

    fp = tmp_path / "doover_config.json"
    TechtopMotorControllerConfig.export(fp, "techtop_motor_controller")
    data = json.loads(fp.read_text())
    assert "config_schema" in data["techtop_motor_controller"]


def test_ui_export(tmp_path):
    from techtop_motor_controller.app_ui import TechtopMotorControllerUI

    fp = tmp_path / "doover_config.json"
    TechtopMotorControllerUI(None, None, None).export(fp, "techtop_motor_controller")
    data = json.loads(fp.read_text())
    children = data["techtop_motor_controller"]["ui_schema"]["children"]
    for name in (
        "start_button",
        "stop_button",
        "reset_button",
        "frequency_setpoint",
        "output_frequency",
    ):
        assert name in children
