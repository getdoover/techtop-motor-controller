from pathlib import Path

from pydoover import ui

from .app_tags import TechtopMotorControllerTags as Tags

SETPOINT_STEP_HZ = 0.5

# Attributes that change at runtime (hidden / display text) are bound to tags:
# the UI schema is published once at setup, so main_loop cannot mutate
# elements. The app updates the hide_* tags each cycle instead and the site
# re-resolves them on every render.


class TechtopMotorControllerUI(ui.UI, display_name=Tags.app_display_name):
    # --- Warnings (top of page; visibility driven by tags) -----------------
    no_comms_warning = ui.WarningIndicator(
        "No communications with drive",
        name="no_comms_warning",
        hidden=Tags.hide_no_comms_warning,
        can_cancel=False,
    )
    trip_warning = ui.WarningIndicator(
        Tags.trip_label,
        name="trip_warning",
        hidden=Tags.hide_trip_warning,
        can_cancel=False,
    )
    not_enabled_warning = ui.WarningIndicator(
        "Drive not enabled: hardware enable (DI1) is open",
        name="not_enabled_warning",
        hidden=Tags.hide_not_enabled_warning,
        can_cancel=False,
    )
    not_modbus_warning = ui.WarningIndicator(
        "Drive not in Modbus control: set P-12 = 3 on the keypad",
        name="not_modbus_warning",
        hidden=Tags.hide_not_modbus_warning,
        can_cancel=False,
    )
    control_disabled_warning = ui.WarningIndicator(
        "Control disabled in configuration (monitor only)",
        name="control_disabled_warning",
        hidden=Tags.hide_control_disabled_warning,
        can_cancel=False,
    )

    # --- Operating values ---------------------------------------------------
    output_frequency = ui.NumericVariable(
        "Output Frequency",
        name="output_frequency",
        value=Tags.output_frequency_hz,
        units="Hz",
        precision=1,
        form=ui.Widget.radial,
        icon="gauge",
    )
    motor_current = ui.NumericVariable(
        "Motor Current",
        name="motor_current",
        value=Tags.motor_current_a,
        units="A",
        precision=1,
        icon="wave-square",
    )
    motor_power = ui.NumericVariable(
        "Motor Power",
        name="motor_power",
        value=Tags.motor_power_kw,
        units="kW",
        precision=2,
        icon="bolt",
    )
    motor_torque = ui.NumericVariable(
        "Motor Torque",
        name="motor_torque",
        value=Tags.torque_pct,
        units="%",
        precision=0,
        icon="rotate",
    )
    drive_state = ui.TextVariable(
        "Drive State",
        name="drive_state",
        value=Tags.drive_state,
        icon="circle-info",
    )

    # --- Control ------------------------------------------------------------
    frequency_setpoint = ui.Slider(
        "Frequency Setpoint",
        name="frequency_setpoint",
        min_val=0,
        max_val=50,
        step_size=SETPOINT_STEP_HZ,
        dual_slider=False,
        inverted=False,
        units="Hz",
        hidden=Tags.hide_setpoint,
        help_str=(
            "Target output frequency. Takes effect immediately while running, "
            "and is used by the next start otherwise."
        ),
        requires_confirm=ui.ConfirmDialog(
            title="Confirm speed change",
            warning_reason="A running motor ramps to the new setpoint immediately.",
        ),
    )
    start_button = ui.Button(
        "Start",
        name="start_button",
        hidden=Tags.hide_start_button,
        colour=ui.Colour.green,
        requires_confirm=ui.ConfirmDialog(
            title="Start Motor",
            warning_reason="Send a run command to the drive.",
            audit=True,
        ),
    )
    stop_button = ui.Button(
        "Stop",
        name="stop_button",
        hidden=Tags.hide_stop_button,
        colour=ui.Colour.red,
        requires_confirm=ui.ConfirmDialog(
            title="Stop Motor",
            warning_reason="The drive will stop the motor.",
            audit=True,
        ),
    )
    reset_button = ui.Button(
        "Reset Trip",
        name="reset_button",
        hidden=Tags.hide_reset_button,
        requires_confirm=ui.ConfirmDialog(
            title="Reset Trip",
            warning_reason="Clear the latched trip on the drive. The motor does not restart.",
            audit=True,
        ),
    )

    # --- Details ------------------------------------------------------------
    details = ui.Submodule(
        "Details",
        name="details",
        is_collapsed=True,
        children=[
            ui.NumericVariable(
                "Setpoint (drive)",
                name="setpoint_readback",
                value=Tags.frequency_setpoint_hz,
                units="Hz",
                precision=1,
            ),
            ui.TextVariable(
                "Control Source",
                name="control_source",
                value=Tags.control_source,
            ),
            ui.TextVariable(
                "Controller State",
                name="controller_state",
                value=Tags.controller_state,
            ),
            ui.TextVariable(
                "Direction",
                name="direction",
                value=Tags.direction,
            ),
            ui.NumericVariable(
                "Output Voltage",
                name="output_voltage",
                value=Tags.output_voltage_v,
                units="V",
                precision=0,
            ),
            ui.NumericVariable(
                "DC Bus Voltage",
                name="dc_bus_voltage",
                value=Tags.dc_bus_voltage_v,
                units="V",
                precision=0,
                # ~540-600 V is normal on a 400/415 V supply.
                ranges=[
                    ui.Range(None, 0, 450, ui.Colour.red),
                    ui.Range(None, 450, 750, ui.Colour.green),
                    ui.Range(None, 750, 900, ui.Colour.red),
                ],
            ),
            ui.NumericVariable(
                "Heatsink Temperature",
                name="heatsink_temp",
                value=Tags.heatsink_temp_c,
                units="°C",
                precision=0,
                ranges=[
                    ui.Range(None, -10, 70, ui.Colour.green),
                    ui.Range(None, 70, 85, ui.Colour.yellow),
                    ui.Range(None, 85, 120, ui.Colour.red),
                ],
            ),
            ui.NumericVariable(
                "Internal Temperature",
                name="internal_temp",
                value=Tags.internal_temp_c,
                units="°C",
                precision=0,
            ),
            ui.NumericVariable(
                "Run Hours",
                name="run_hours",
                value=Tags.run_hours,
                units="h",
                precision=1,
            ),
            ui.NumericVariable(
                "Energy",
                name="energy",
                value=Tags.energy_kwh,
                units="kWh",
                precision=1,
            ),
            ui.NumericVariable(
                "Drive Max Frequency (P-01)",
                name="drive_max_frequency",
                value=Tags.drive_max_frequency_hz,
                units="Hz",
                precision=1,
            ),
            ui.NumericVariable(
                "Motor Rated Current (P-08)",
                name="motor_rated_current",
                value=Tags.motor_rated_current_a,
                units="A",
                precision=1,
            ),
            ui.BooleanVariable("Enable (DI1)", name="di1", value=Tags.di1),
            ui.BooleanVariable("Digital Input 2", name="di2", value=Tags.di2),
            ui.BooleanVariable("Digital Input 3", name="di3", value=Tags.di3),
            ui.BooleanVariable("Digital Input 4", name="di4", value=Tags.di4),
            ui.BooleanVariable(
                "Relay Closed", name="relay_closed", value=Tags.relay_closed
            ),
            ui.BooleanVariable("At Speed", name="at_speed", value=Tags.at_speed),
            ui.BooleanVariable("Mains Loss", name="mains_loss", value=Tags.mains_loss),
            ui.BooleanVariable("Overload", name="overload", value=Tags.overload),
        ],
    )

    async def setup(self):
        # The slider's ends follow the configured limits.
        self.frequency_setpoint.min_val = float(self.config.min_frequency_hz.value)
        self.frequency_setpoint.max_val = float(self.config.max_frequency_hz.value)


def export():
    TechtopMotorControllerUI(None, None, None).export(
        Path(__file__).parents[2] / "doover_config.json",
        "techtop_motor_controller",
    )


if __name__ == "__main__":
    export()
