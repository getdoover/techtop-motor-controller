from pathlib import Path

from pydoover import config
from pydoover.config import ApplicationPosition
from pydoover.docker.modbus import ModbusConfig


class DriveModbusConfig(ModbusConfig):
    """pydoover's Modbus bus definition with the E3's factory serial defaults.

    The Optidrive E3 ships at 115.2 kbps, 8 data bits, no parity, 1 stop bit
    (P-36 index 2). pydoover's own default of 9600 would never answer.
    """

    def __init__(self, display_name: str = "Modbus Config"):
        # Object supports Django-style child overrides (``child__attr=value``);
        # ModbusConfig's own __init__ does not pass kwargs through, so call
        # Object's directly. Redeclaring the fields would be a duplicate name.
        config.Object.__init__(
            self,
            display_name,
            serial_baud__default=115200,
            serial_timeout__default=0.5,
        )


class NotificationsConfig(config.Object):
    on_trip = config.Boolean(
        "Notify on Trip",
        default=True,
        description="Send a notification when the drive trips (with the trip code).",
    )
    on_comms_loss = config.Boolean(
        "Notify on Comms Loss",
        default=True,
        description="Send a notification when the drive stops answering on Modbus.",
    )
    on_start = config.Boolean(
        "Notify on Start",
        default=False,
        description="Send a notification each time the motor starts.",
    )
    on_stop = config.Boolean(
        "Notify on Stop",
        default=False,
        description="Send a notification each time the motor stops.",
    )


class TechtopMotorControllerConfig(config.Schema):
    # --- Bus -----------------------------------------------------------------
    # The attribute must be called `modbus_config`: pydoover's ModbusInterface
    # picks it up by that name to know which bus reads/writes go to.
    modbus_config = DriveModbusConfig()

    modbus_unit_id = config.Integer(
        "Modbus Unit ID",
        default=1,
        minimum=0,
        maximum=63,
        description="Drive Modbus address (P-36 index 1 on the drive). Factory default is 1.",
    )

    # --- Hardware enable ----------------------------------------------------
    enable_output_pin = config.Integer(
        "Enable Output Pin",
        default=None,
        minimum=0,
        description=(
            "Doovit digital output wired to drive terminal 2 (Digital Input 1, the "
            "hardware enable). The app holds it high while running and drops it on "
            "shutdown. Leave blank if the enable is wired some other way, e.g. a "
            "link from terminal 1 to 2."
        ),
    )

    # --- Control -------------------------------------------------------------
    control_enabled = config.Boolean(
        "Control Enabled",
        default=True,
        description=(
            "Allow this app to start, stop and set the speed of the drive (from the "
            "UI and over RPC). Off = monitor only; nothing is ever written to the drive."
        ),
    )
    max_frequency_hz = config.Number(
        "Max Frequency (Hz)",
        default=50.0,
        minimum=0.0,
        description=(
            "Highest setpoint this app will send. Also capped by the drive's own "
            "P-01; a setpoint above P-01 is rejected by the drive."
        ),
    )
    min_frequency_hz = config.Number(
        "Min Frequency (Hz)",
        default=0.0,
        minimum=0.0,
        description="Lowest setpoint this app will send.",
    )
    default_frequency_hz = config.Number(
        "Default Frequency (Hz)",
        default=50.0,
        minimum=0.0,
        description="Setpoint used for a start command that does not name a frequency.",
    )
    stop_mode = config.Enum(
        "Stop Mode",
        choices=["ramp", "fast", "coast"],
        default="ramp",
        description=(
            "How a stop is performed: ramp down on the P-04 deceleration ramp, "
            "fast stop on the P-24 ramp, or coast (output disabled immediately)."
        ),
    )
    start_timeout_s = config.Number(
        "Start Timeout (s)",
        default=10.0,
        minimum=1.0,
        description="Seconds to wait for the drive to report Running after a start before giving up.",
    )
    stop_timeout_s = config.Number(
        "Stop Timeout (s)",
        default=60.0,
        minimum=1.0,
        description="Seconds to wait for the drive to report stopped before warning.",
    )

    # --- Polling -------------------------------------------------------------
    poll_interval_s = config.Number(
        "Poll Interval (s)",
        default=1.0,
        minimum=0.2,
        description=(
            "How often the drive is polled and, while under Modbus control, the "
            "control word is refreshed. Keep this well under the drive's Modbus "
            "watchdog (P-36 index 3) if that is enabled."
        ),
    )
    parameter_refresh_s = config.Number(
        "Parameter Refresh (s)",
        default=60.0,
        minimum=5.0,
        description="How often the drive's parameters (control source, limits, motor rating) are re-read.",
    )
    comms_loss_timeout_s = config.Number(
        "Comms Loss Timeout (s)",
        default=30.0,
        minimum=1.0,
        description="Seconds without a Modbus reply before the drive is reported as disconnected.",
    )

    notifications = NotificationsConfig(
        "Notifications",
        description="Which drive events post to the notifications channel.",
    )

    position = ApplicationPosition()

    # --- Derived helpers -----------------------------------------------------

    @property
    def enable_pin(self) -> int | None:
        value = self.enable_output_pin.value
        return None if value is None else int(value)

    def clamp_frequency(self, frequency_hz: float) -> float:
        low = float(self.min_frequency_hz.value)
        high = float(self.max_frequency_hz.value)
        return max(low, min(high, float(frequency_hz)))


def export():
    TechtopMotorControllerConfig.export(
        Path(__file__).parents[2] / "doover_config.json",
        "techtop_motor_controller",
    )


if __name__ == "__main__":
    export()
