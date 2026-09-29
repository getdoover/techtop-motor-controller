"""Techtop TTA-3 (Invertek Optidrive E3) motor controller.

All Modbus traffic goes through pydoover's built-in Modbus interface
(``self.modbus_iface``), so the bus -- RS-485 on the Doovit's own port, a USB
adapter, or TCP -- is purely a matter of the app's Modbus config.

Control model
-------------
The drive's own state machine (Not Ready / Ready / Running / Tripped) is
authoritative; :class:`MotorController` sequences commands against it. Every
poll the app:

1. reads the drive's status block,
2. lets the controller advance its state,
3. while the drive is in Modbus control mode, rewrites the setpoint and control
   word so the drive's Modbus watchdog (P-36 index 3), if enabled, stays fed,
4. publishes tags and drives the UI's conditional visibility.

Peer apps command this app over RPC (``start`` / ``stop`` / ``set_frequency`` /
``reset_fault`` / ``get_status``); the UI buttons go through the same code path.
The commissioning panel uses ``get_diagnostics``, ``read_parameters`` and
``write_parameter`` on the same channel.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from datetime import datetime

from pydoover import ui
from pydoover.docker import Application
from pydoover.rpc import DEFAULT_CHANNEL as RPC_CHANNEL
from pydoover.rpc import RPCError
from pydoover.rpc import handler as rpc_handler

from .app_config import TechtopMotorControllerConfig
from .app_state import (
    DISCONNECTED,
    NOT_READY,
    STOP_MODES,
    TRIPPED,
    MotorController,
)
from .app_tags import TechtopMotorControllerTags
from .app_ui import TechtopMotorControllerUI
from .drive import (
    NAMEPLATE_PARAMETERS,
    P01_MAX_FREQUENCY,
    P02_MIN_FREQUENCY,
    DriveParameters,
    DriveStatus,
    TechtopDrive,
    nameplate_raw,
)
from .parameters import (
    FORBIDDEN_PARAMETERS,
    P36_SERIAL_COMMS,
    PARAMETERS,
    PARAMETERS_BY_NUMBER,
    ParameterSpec,
    decode_p36,
    decode_value,
    is_step_multiple,
    parse_parameter_id,
)

log = logging.getLogger(__name__)

DEFAULT_DISPLAY_NAME = "Techtop Motor Controller"

# The data plane deserialises severity by variant name.
SEVERITY_INFO = "Info"
SEVERITY_WARN = "Warn"

METER_REFRESH_S = 30.0
SUMMARY_LOG_INTERVAL_S = 60.0
DIRECTIONS = ("forward", "reverse")

CONFIG_MANAGED_DESCRIPTION = "Set in the app config (nameplate)"
NAMEPLATE_FIELDS = {number: field for number, field, _ in NAMEPLATE_PARAMETERS}


class TechtopMotorControllerApplication(Application):
    config_cls = TechtopMotorControllerConfig
    tags_cls = TechtopMotorControllerTags
    ui_cls = TechtopMotorControllerUI

    config: TechtopMotorControllerConfig
    tags: TechtopMotorControllerTags
    ui: TechtopMotorControllerUI

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def setup(self):
        # `transitions` logs every callback at INFO; the controller logs the
        # transitions that matter itself.
        logging.getLogger("transitions").setLevel(logging.WARNING)

        cfg = self.config
        self.loop_target_period = float(cfg.poll_interval_s.value)

        self.drive = TechtopDrive(
            self.modbus_iface, unit_id=int(cfg.modbus_unit_id.value)
        )
        self.controller = MotorController(
            start_timeout_s=float(cfg.start_timeout_s.value),
            stop_timeout_s=float(cfg.stop_timeout_s.value),
            default_stop_mode=str(cfg.stop_mode.value),
            on_state_change=self._on_controller_state_change,
        )
        self.controller.setpoint_hz = cfg.clamp_frequency(
            cfg.default_frequency_hz.value
        )
        self._restore_setpoint_from_ui()

        self.params = DriveParameters()
        self._params_read_at: float | None = None
        self._meters_read_at: float | None = None
        self._last_contact: float | None = None
        self._comms_lost_notified = False
        self._prev_running: bool | None = None
        self._prev_trip: tuple[bool, int] | None = None
        self._tick_lock = asyncio.Lock()
        self._last_summary_log: float = 0.0
        # Nameplate values the drive refused, so a bad config value is reported
        # once rather than rewritten every parameter refresh.
        self._nameplate_refused: dict[int, int] = {}

        log.info(
            "Techtop motor controller ready: unit %s, control %s, enable pin %s",
            cfg.modbus_unit_id.value,
            "enabled" if self.control_enabled else "disabled (monitor only)",
            cfg.enable_pin,
        )

    def _restore_setpoint_from_ui(self):
        """Carry the operator's last slider position across a restart."""
        try:
            value = self.ui.frequency_setpoint.value
        except (KeyError, AttributeError, TypeError):
            return
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            self.controller.setpoint_hz = self.config.clamp_frequency(value)

    async def main_loop(self):
        await self._tick()

    async def on_shutdown_at(self, dt: datetime) -> None:
        """The Doovit is about to power down: stop the motor and drop the enable."""
        log.warning("Shutdown scheduled at %s: stopping the drive", dt)
        if self.control_allowed and self.controller.can_stop:
            self.controller.request_stop()
            try:
                await self._tick()
            except Exception as e:  # noqa: BLE001 - the shutdown hook must never raise
                log.warning("Stop on shutdown failed: %s", e)
        await self._assert_enable(False)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def control_enabled(self) -> bool:
        return bool(self.config.control_enabled.value)

    @property
    def control_allowed(self) -> bool:
        """This app may command the drive: control enabled and P-12 in Modbus mode."""
        return self.control_enabled and self.params.modbus_control

    @property
    def contactable(self) -> bool:
        return (
            self.controller.state != DISCONNECTED and self.drive.last_status.contactable
        )

    # ------------------------------------------------------------------
    # The poll / control cycle
    # ------------------------------------------------------------------

    async def _tick(self):
        async with self._tick_lock:
            await self._tick_locked()

    async def _tick_locked(self):
        cfg = self.config
        now = time.monotonic()

        # The enable output is only asserted once the drive is known to be in
        # Modbus control mode with control enabled. In terminal mode (P-12 = 0)
        # DI1 is the RUN command, so holding it high there would start the
        # motor; it also locks parameter edits on the keypad while enabled.
        await self._assert_enable(self.control_allowed)

        include_meters = (
            self._meters_read_at is None or now - self._meters_read_at > METER_REFRESH_S
        )
        status = await self.drive.read_status(include_meters=include_meters)

        if status.contactable:
            self._last_contact = now
            if include_meters:
                self._meters_read_at = now
            if self._params_read_at is None or now - self._params_read_at > float(
                cfg.parameter_refresh_s.value
            ):
                await self._refresh_parameters(now)
        else:
            # One missed poll is not an outage. Hold state until the grace
            # period passes, then let the controller see the disconnect.
            grace = float(cfg.comms_loss_timeout_s.value)
            if self._last_contact is not None and now - self._last_contact < grace:
                log.warning(
                    "Drive did not answer; %.0fs until reported disconnected",
                    grace - (now - self._last_contact),
                )
                return

        state = await self.controller.spin(status, control_allowed=self.control_allowed)

        if status.contactable and self.control_allowed:
            ok = await self.drive.write_command(
                self.controller.control_word(), self.controller.signed_setpoint_hz()
            )
            if not ok:
                log.warning(
                    "Control word / setpoint write was not acknowledged by the drive"
                )

        await self._update_tags(status, state)
        await self._check_notifications(status, state)
        self._log_summary(status, state, now)

    def _log_summary(self, status: DriveStatus, state: str, now: float):
        """A one-line heartbeat so `docker logs` shows the loop is alive."""
        if now - self._last_summary_log < SUMMARY_LOG_INTERVAL_S:
            return
        self._last_summary_log = now
        if not status.contactable:
            log.info("Drive: no comms (controller %s)", state)
            return
        log.info(
            "Drive %s (controller %s, P-12 %s): out %.1f Hz, set %s Hz, %.1f A, %.2f kW, DC %.0f V, "
            "heatsink %.0f C, DI1 %s, trip %s, cw 0x%02x",
            status.state_name,
            state,
            self.params.control_source,
            status.output_frequency_hz,
            status.frequency_setpoint_hz,
            status.motor_current_a,
            status.motor_power_kw,
            status.dc_bus_voltage_v,
            status.heatsink_temp_c,
            "on" if status.di1 else "off",
            status.trip_code if status.tripped else "-",
            self.controller.control_word(),
        )

    async def _refresh_parameters(self, now: float):
        params = await self.drive.read_parameters()
        if params.control_source_code is None:
            return
        if params.control_source != self.params.control_source:
            log.info("Drive control source (P-12): %s", params.control_source)
        self.params = params
        self._params_read_at = now
        if await self._apply_nameplate(params):
            reread = await self.drive.read_parameters()
            if reread.control_source_code is not None:
                self.params = reread

    async def _apply_nameplate(self, params: DriveParameters) -> bool:
        """Write configured motor nameplate values (P-07..P-10) the drive does
        not already hold. Only while control is enabled and the motor is
        stopped. Returns True if anything was written."""
        wanted = self.config.nameplate
        if not wanted or not self.control_enabled:
            return False
        if self.drive.last_status.running:
            return False
        wrote = False
        for number, field, scale in NAMEPLATE_PARAMETERS:
            if field not in wanted:
                continue
            target = nameplate_raw(wanted[field], scale)
            current = getattr(params, field)
            if current is not None and nameplate_raw(current, scale) == target:
                continue
            if self._nameplate_refused.get(number) == target:
                continue
            try:
                ok = await self.drive.write_parameter(number, target)
            except Exception as e:  # noqa: BLE001 - e.g. a modbus interface without writeSingleRegister
                log.warning("Could not write P-%02d: %s", number, e)
                ok = False
            if ok:
                log.info("Set P-%02d to %s (was %s)", number, wanted[field], current)
                self._nameplate_refused.pop(number, None)
                wrote = True
            else:
                log.warning(
                    "Drive refused P-%02d = %s (out of range for this drive?)",
                    number,
                    wanted[field],
                )
                self._nameplate_refused[number] = target
        return wrote

    async def _assert_enable(self, high: bool):
        pin = self.config.enable_pin
        if pin is None:
            return
        try:
            await self.platform_iface.set_do(pin, bool(high))
        except Exception as e:  # noqa: BLE001 - a flaky DO must not stop the poll loop
            log.warning("Could not set enable output DO%s: %s", pin, e)

    # ------------------------------------------------------------------
    # Commands (shared by UI handlers and RPC)
    # ------------------------------------------------------------------

    def _require_control(self):
        if not self.control_enabled:
            raise RPCError(
                "CONTROL_DISABLED", "Control is disabled in this app's configuration"
            )
        if not self.contactable:
            raise RPCError("NOT_CONNECTED", "No communications with the drive")
        if not self.params.modbus_control:
            source = self.params.control_source or "unknown"
            raise RPCError(
                "NOT_MODBUS_CONTROL",
                f"Drive control source is '{source}'; set P-12 = 3 on the drive for Modbus control",
            )

    def _validate_frequency(self, frequency_hz) -> float:
        try:
            value = float(frequency_hz)
        except (TypeError, ValueError):
            raise RPCError(
                "INVALID_FREQUENCY",
                f"frequency_hz must be a number, got {frequency_hz!r}",
            )
        if not math.isfinite(value) or value < 0:
            raise RPCError(
                "INVALID_FREQUENCY", "frequency_hz must be a non-negative number"
            )
        value = self.config.clamp_frequency(value)
        # The drive rejects (Modbus exception) any setpoint above its own P-01.
        drive_max = self.params.max_frequency_hz
        if drive_max is not None and value > drive_max:
            value = drive_max
        return round(value, 1)

    @staticmethod
    def _validate_direction(direction) -> bool | None:
        if direction is None:
            return None
        if direction not in DIRECTIONS:
            raise RPCError(
                "INVALID_DIRECTION", f"direction must be one of {DIRECTIONS}"
            )
        return direction == "reverse"

    async def command_start(
        self, frequency_hz=None, direction=None, *, source: str = "ui"
    ):
        self._require_control()
        state = self.controller.state
        if state == TRIPPED:
            desc = self.drive.last_status.trip_description or "unknown trip"
            raise RPCError("TRIPPED", f"Drive is tripped ({desc}); reset it first")
        if state == NOT_READY:
            raise RPCError(
                "NOT_READY",
                "Drive is not ready: hardware enable (DI1) open, mains loss, or drive in standby",
            )

        if frequency_hz is not None:
            self.controller.setpoint_hz = self._validate_frequency(frequency_hz)
        reverse = self._validate_direction(direction)
        if reverse is not None:
            self.controller.reverse = reverse

        self.controller.request_start()
        await self.tags.last_command.set(
            f"start {self.controller.signed_setpoint_hz():g} Hz ({source})"
        )
        await self._tick()

    async def command_stop(self, mode: str | None = None, *, source: str = "ui"):
        self._require_control()
        if mode is not None and mode not in STOP_MODES:
            raise RPCError("INVALID_STOP_MODE", f"mode must be one of {STOP_MODES}")
        self.controller.request_stop(mode)
        await self.tags.last_command.set(f"stop {self.controller.stop_mode} ({source})")
        await self._tick()

    async def command_set_frequency(
        self, frequency_hz, direction=None, *, source: str = "ui"
    ):
        self._require_control()
        value = self._validate_frequency(frequency_hz)
        reverse = self._validate_direction(direction)
        self.controller.setpoint_hz = value
        if reverse is not None:
            self.controller.reverse = reverse
        await self.tags.last_command.set(
            f"setpoint {self.controller.signed_setpoint_hz():g} Hz ({source})"
        )
        if source != "ui":
            # Keep the operator's slider in step with what a peer app asked for.
            await self.ui.frequency_setpoint.set(value)
        await self._tick()

    async def command_reset(self, *, source: str = "ui"):
        self._require_control()
        if self.controller.state != TRIPPED:
            raise RPCError("NOT_TRIPPED", "Drive is not tripped")
        self.controller.request_reset()
        await self.tags.last_command.set(f"reset ({source})")
        await self._tick()

    def status_dict(self) -> dict:
        status = self.drive.last_status
        return {
            "comms_active": self.contactable,
            "controller_state": self.controller.state,
            "drive_state": status.state_name,
            "control_source": self.params.control_source,
            "modbus_control": self.params.modbus_control,
            "control_enabled": self.control_enabled,
            "running": status.running,
            "ready": status.ready,
            "tripped": status.tripped,
            "trip_code": status.trip_code if status.tripped else None,
            "trip_description": status.trip_description if status.tripped else None,
            "enable_present": status.enable_present,
            "requested_frequency_hz": self.controller.signed_setpoint_hz(),
            "drive_setpoint_hz": status.frequency_setpoint_hz,
            "output_frequency_hz": status.output_frequency_hz,
            "motor_current_a": status.motor_current_a,
            "motor_power_kw": status.motor_power_kw,
            "torque_pct": round(status.torque_pct, 1),
            "dc_bus_voltage_v": status.dc_bus_voltage_v,
            "direction": "reverse" if self.controller.reverse else "forward",
        }

    # ------------------------------------------------------------------
    # RPC surface for peer apps
    # ------------------------------------------------------------------
    #
    # From another app:  await self.rpc.call("start", params={"frequency_hz": 30},
    #                                        app_key="techtop_motor_controller_1")
    #
    # The channel must be named: RPCManager only subscribes to a handler's
    # channel when one is given, so a channel-less handler never hears the
    # default `dv-rpc` channel that `rpc.call()` publishes on.

    @staticmethod
    def _rpc_source(ctx) -> str:
        actor = getattr(ctx, "actor", None)
        if isinstance(actor, dict) and actor.get("name"):
            return f"rpc:{actor['name']}"
        return "rpc"

    @rpc_handler("start", channel=RPC_CHANNEL)
    async def rpc_start(self, ctx, payload):
        payload = payload or {}
        await self.command_start(
            payload.get("frequency_hz"),
            payload.get("direction"),
            source=self._rpc_source(ctx),
        )
        return self.status_dict()

    @rpc_handler("stop", channel=RPC_CHANNEL)
    async def rpc_stop(self, ctx, payload):
        payload = payload or {}
        await self.command_stop(payload.get("mode"), source=self._rpc_source(ctx))
        return self.status_dict()

    @rpc_handler("set_frequency", channel=RPC_CHANNEL)
    async def rpc_set_frequency(self, ctx, payload):
        payload = payload or {}
        if "frequency_hz" not in payload:
            raise RPCError("INVALID_FREQUENCY", "frequency_hz is required")
        await self.command_set_frequency(
            payload["frequency_hz"],
            payload.get("direction"),
            source=self._rpc_source(ctx),
        )
        return self.status_dict()

    @rpc_handler("reset_fault", channel=RPC_CHANNEL)
    async def rpc_reset_fault(self, ctx, payload):
        await self.command_reset(source=self._rpc_source(ctx))
        return self.status_dict()

    @rpc_handler("get_status", channel=RPC_CHANNEL)
    async def rpc_get_status(self, ctx, payload):
        return self.status_dict()

    # ------------------------------------------------------------------
    # Commissioning RPCs (diagnostics and drive parameters)
    # ------------------------------------------------------------------

    @staticmethod
    def _rpc_actor(ctx) -> str:
        """Who issued an RPC, for the audit log."""
        actor = getattr(ctx, "actor", None)
        if isinstance(actor, dict):
            for key in ("name", "email", "id"):
                if actor.get(key):
                    return str(actor[key])
        return "unknown"

    @rpc_handler("get_diagnostics", channel=RPC_CHANNEL)
    async def rpc_get_diagnostics(self, ctx, payload):
        return await self.diagnostics()

    async def diagnostics(self) -> dict:
        """One fresh read of the status block, setpoint and meters."""
        async with self._tick_lock:
            status = await self.drive.read_status(include_meters=True)
        if not status.contactable:
            return {
                "output_hz": None,
                "output_current_a": None,
                "motor_rpm": None,
                "dc_bus_v": None,
                "heatsink_c": None,
                "drive_state": status.state_name,
                "trip_code": None,
                "trip_description": None,
                "run_hours": None,
                "recent_trips": None,
                "comms_ok": False,
            }
        return {
            "output_hz": status.output_frequency_hz,
            "output_current_a": status.motor_current_a,
            "motor_rpm": self._estimated_rpm(status.output_frequency_hz),
            "dc_bus_v": status.dc_bus_voltage_v,
            "heatsink_c": status.heatsink_temp_c,
            "drive_state": status.state_name,
            "trip_code": status.trip_code if status.tripped else None,
            "trip_description": status.trip_description if status.tripped else None,
            "run_hours": None
            if status.run_hours is None
            else round(status.run_hours, 2),
            # The E3 keeps its trip log (P00-13) on the keypad only; it is not
            # in the Modbus register map.
            "recent_trips": None,
            "comms_ok": True,
        }

    def _estimated_rpm(self, output_hz: float) -> float | None:
        """The drive does not report shaft speed over Modbus. With the nameplate
        speed and frequency known (P-10 > 0), scale the output frequency by
        them, as the drive's own display does; otherwise None."""
        rated_rpm = self.params.motor_rated_speed_rpm
        rated_hz = self.params.motor_rated_frequency_hz
        if not rated_rpm or not rated_hz:
            return None
        return round(output_hz * rated_rpm / rated_hz)

    def _config_managed(self, number: int) -> bool:
        """A nameplate parameter the app config sets (and would rewrite)."""
        field = NAMEPLATE_FIELDS.get(number)
        return field is not None and field in self.config.nameplate

    def _parameter_limits(
        self, spec: ParameterSpec, raw: dict[int, int]
    ) -> tuple[float | None, float | None]:
        """Range for a parameter, resolving the P-01 / P-02 cross limits from
        the drive's current values."""
        minimum, maximum = spec.minimum, spec.maximum
        if spec.number == P01_MAX_FREQUENCY and P02_MIN_FREQUENCY in raw:
            minimum = PARAMETERS_BY_NUMBER[P02_MIN_FREQUENCY].from_raw(
                raw[P02_MIN_FREQUENCY]
            )
        if spec.number == P02_MIN_FREQUENCY and P01_MAX_FREQUENCY in raw:
            maximum = PARAMETERS_BY_NUMBER[P01_MAX_FREQUENCY].from_raw(
                raw[P01_MAX_FREQUENCY]
            )
        return minimum, maximum

    @rpc_handler("read_parameters", channel=RPC_CHANNEL)
    async def rpc_read_parameters(self, ctx, payload):
        return {"parameters": await self.read_parameter_list()}

    async def read_parameter_list(self) -> list[dict]:
        async with self._tick_lock:
            raw = await self.drive.read_parameter_values(
                spec.number for spec in PARAMETERS
            )
        entries = []
        for spec in PARAMETERS:
            minimum, maximum = self._parameter_limits(spec, raw)
            writable = spec.writable and self.control_enabled
            description = spec.description
            if spec.number == P36_SERIAL_COMMS and spec.number in raw:
                address, baud, trip = decode_p36(raw[spec.number])
                description = (
                    f"Drive Modbus address {address}, {baud:g} kbps, comms loss "
                    f"{trip} (P-36). Set on the drive keypad only."
                )
            if spec.writable and self._config_managed(spec.number):
                writable = False
                description = CONFIG_MANAGED_DESCRIPTION
            entries.append(
                {
                    "id": spec.id,
                    "name": spec.name,
                    "value": decode_value(spec, raw.get(spec.number)),
                    "units": spec.units,
                    "min": minimum,
                    "max": maximum,
                    "step": spec.step,
                    "writable": writable,
                    "stop_required": spec.stop_required,
                    "description": description,
                }
            )
        return entries

    @rpc_handler("write_parameter", channel=RPC_CHANNEL)
    async def rpc_write_parameter(self, ctx, payload):
        payload = payload or {}
        return await self.write_parameter(
            payload.get("parameter"), payload.get("value"), actor=self._rpc_actor(ctx)
        )

    def _writable_spec(self, parameter) -> ParameterSpec:
        number = parse_parameter_id(parameter)
        if number in FORBIDDEN_PARAMETERS:
            raise RPCError("NOT_ALLOWED", FORBIDDEN_PARAMETERS[number])
        spec = PARAMETERS_BY_NUMBER.get(number)
        if spec is None or not spec.writable:
            raise RPCError(
                "NOT_ALLOWED",
                f"{parameter!r} is not a parameter this panel can change",
            )
        if not self.control_enabled:
            raise RPCError(
                "NOT_ALLOWED",
                "Control is disabled in this app's configuration (monitor only)",
            )
        if self._config_managed(number):
            raise RPCError(
                "NOT_ALLOWED",
                f"{spec.id}: {CONFIG_MANAGED_DESCRIPTION}. Change it there, or clear "
                "it in the config to set it from the panel.",
            )
        return spec

    @staticmethod
    def _check_value(spec: ParameterSpec, value, minimum, maximum) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RPCError("OUT_OF_RANGE", f"{spec.id} value must be a number")
        value = float(value)
        if not math.isfinite(value):
            raise RPCError("OUT_OF_RANGE", f"{spec.id} value must be a finite number")
        if (minimum is not None and value < minimum) or (
            maximum is not None and value > maximum
        ):
            low = "-" if minimum is None else f"{minimum:g}"
            high = "drive rating" if maximum is None else f"{maximum:g}"
            raise RPCError(
                "OUT_OF_RANGE",
                f"{spec.id} must be between {low} and {high} {spec.units}".rstrip(),
            )
        if not is_step_multiple(value, spec.step):
            raise RPCError(
                "OUT_OF_RANGE", f"{spec.id} must be a multiple of {spec.step:g}"
            )
        return value

    async def write_parameter(self, parameter, value, *, actor: str = "ui") -> dict:
        """Range-check, scale, write (FC06), read back and compare one parameter.

        Raises ``RPCError`` with DRIVE_RUNNING, OUT_OF_RANGE, NOT_ALLOWED,
        READBACK_MISMATCH or COMMS_ERROR.
        """
        spec = self._writable_spec(parameter)
        async with self._tick_lock:
            status = await self.drive.read_status()
            if not status.contactable:
                raise RPCError("COMMS_ERROR", "No communications with the drive")
            if spec.stop_required and status.running:
                raise RPCError(
                    "DRIVE_RUNNING", f"Stop the motor before changing {spec.id}"
                )

            limits_raw = await self.drive.read_parameter_values(
                (P01_MAX_FREQUENCY, P02_MIN_FREQUENCY)
            )
            minimum, maximum = self._parameter_limits(spec, limits_raw)
            number_value = self._check_value(spec, value, minimum, maximum)
            target = spec.to_raw(number_value)
            before = await self.drive.read_parameter(spec.number)

            try:
                ok = await self.drive.write_parameter(spec.number, target)
            except Exception as e:
                log.warning("Write of %s failed: %s", spec.id, e)
                raise RPCError("COMMS_ERROR", f"Write of {spec.id} failed: {e}") from e
            if not ok:
                # A refused write and a lost reply look the same here; if the
                # drive still answers, it refused the value.
                if await self.drive.read_parameter(spec.number) is None:
                    raise RPCError(
                        "COMMS_ERROR", f"No reply from the drive writing {spec.id}"
                    )
                raise RPCError(
                    "OUT_OF_RANGE",
                    f"The drive refused {spec.id} = {number_value:g} {spec.units}".rstrip(),
                )

            readback = await self.drive.read_parameter(spec.number)
            if readback is None:
                raise RPCError(
                    "COMMS_ERROR", f"No reply from the drive reading back {spec.id}"
                )
            if readback != target:
                raise RPCError(
                    "READBACK_MISMATCH",
                    f"Wrote {spec.id} = {number_value:g} but the drive reads back "
                    f"{spec.from_raw(readback):g}",
                )

        result = spec.from_raw(readback)
        log.info(
            "Parameter %s set to %s %s (raw %s, was %s) by %s",
            spec.id,
            result,
            spec.units,
            readback,
            None if before is None else spec.from_raw(before),
            actor,
        )
        await self.tags.last_command.set(f"{spec.id} = {result:g} ({actor})")
        # Pick up new limits / nameplate on the next poll.
        self._params_read_at = None
        return {"parameter": spec.id, "value": result}

    # ------------------------------------------------------------------
    # UI handlers
    # ------------------------------------------------------------------

    @ui.handler("start_button")
    async def on_start_button(self, ctx, value):
        await ctx.set_value(None)
        await self.command_start(source="ui")

    @ui.handler("stop_button")
    async def on_stop_button(self, ctx, value):
        await ctx.set_value(None)
        await self.command_stop(source="ui")

    @ui.handler("reset_button")
    async def on_reset_button(self, ctx, value):
        await ctx.set_value(None)
        await self.command_reset(source="ui")

    @ui.handler("frequency_setpoint")
    async def on_frequency_setpoint(self, ctx, value):
        if value is None:
            return
        await self.command_set_frequency(value, source="ui")

    # ------------------------------------------------------------------
    # Tags, UI visibility, notifications
    # ------------------------------------------------------------------

    async def _on_controller_state_change(self, old: str, new: str):
        await self.tags.controller_state.set(new)

    @property
    def display_name(self) -> str:
        # APP_DISPLAY_NAME only arrives with a platform deployment config; a
        # hand-run (CONFIG_FP) leaves it blank.
        return self.app_display_name or DEFAULT_DISPLAY_NAME

    async def _update_tags(self, status: DriveStatus, state: str):
        tags = self.tags
        contactable = status.contactable
        params = self.params
        ctl = self.controller

        await tags.comms_active.set(contactable)
        await tags.controller_state.set(state)
        await tags.drive_state.set(status.state_name)
        await tags.control_source.set(params.control_source)
        await tags.modbus_control.set(params.modbus_control)
        await tags.drive_max_frequency_hz.set(params.max_frequency_hz)
        await tags.motor_rated_current_a.set(params.motor_rated_current_a)

        if contactable:
            await tags.running.set(status.running)
            await tags.ready.set(status.ready)
            await tags.tripped.set(status.tripped)
            await tags.trip_code.set(status.trip_code if status.tripped else None)
            await tags.trip_description.set(
                status.trip_description if status.tripped else None
            )
            await tags.enable_present.set(status.enable_present)
            await tags.at_speed.set(status.at_speed)
            await tags.mains_loss.set(status.mains_loss)
            await tags.overload.set(status.overload)
            direction = (
                "reverse"
                if (status.reverse if status.running else ctl.reverse)
                else "forward"
            )
            await tags.direction.set(direction)

            await tags.frequency_setpoint_hz.set(status.frequency_setpoint_hz)
            await tags.output_frequency_hz.set(status.output_frequency_hz)
            await tags.motor_current_a.set(status.motor_current_a)
            await tags.motor_power_kw.set(status.motor_power_kw)
            await tags.torque_pct.set(round(status.torque_pct, 1))
            await tags.output_voltage_v.set(status.output_voltage_v)
            await tags.dc_bus_voltage_v.set(status.dc_bus_voltage_v)
            await tags.heatsink_temp_c.set(status.heatsink_temp_c)
            await tags.internal_temp_c.set(status.internal_temp_c)
            if status.energy_kwh is not None:
                await tags.energy_kwh.set(round(status.energy_kwh, 1))
            if status.run_hours is not None:
                await tags.run_hours.set(round(status.run_hours, 2))

            await tags.di1.set(status.di1)
            await tags.di2.set(status.di2)
            await tags.di3.set(status.di3)
            await tags.di4.set(status.di4)
            await tags.relay_closed.set(status.relay_closed)
        else:
            await tags.running.set(False)
            await tags.ready.set(False)

        # Keep the panel title fixed; the drive state has its own row.
        await tags.app_display_name.set(self.display_name)
        await self._update_visibility(status, state)

    async def _update_visibility(self, status: DriveStatus, state: str):
        tags = self.tags
        contactable = status.contactable
        control_allowed = self.control_allowed
        ctl = self.controller
        known_source = self.params.control_source_code is not None

        await tags.hide_no_comms_warning.set(contactable)
        await tags.hide_trip_warning.set(not (contactable and status.tripped))
        if contactable and status.tripped:
            await tags.trip_label.set(
                f"Drive tripped: {status.trip_description} (code {status.trip_code})"
            )
        await tags.hide_not_enabled_warning.set(
            not (contactable and not status.tripped and not status.enable_present)
        )
        await tags.hide_not_modbus_warning.set(
            not (
                contactable
                and self.control_enabled
                and known_source
                and not self.params.modbus_control
            )
        )
        await tags.hide_control_disabled_warning.set(self.control_enabled)

        await tags.hide_start_button.set(
            not (contactable and control_allowed and ctl.can_start)
        )
        await tags.hide_stop_button.set(
            not (contactable and control_allowed and ctl.can_stop)
        )
        await tags.hide_reset_button.set(
            not (contactable and control_allowed and ctl.can_reset)
        )
        await tags.hide_setpoint.set(not (contactable and control_allowed))

    async def _notify(self, message: str, severity: str = SEVERITY_WARN):
        try:
            await self.create_message(
                "notifications",
                {
                    "message": f"{self.display_name}: {message}",
                    "severity": severity,
                },
            )
        except Exception as e:  # noqa: BLE001 - notifications are best effort
            log.warning("Notification failed: %s", e)

    async def _check_notifications(self, status: DriveStatus, state: str):
        notif = self.config.notifications

        # Comms loss / restore
        if state == DISCONNECTED:
            if not self._comms_lost_notified:
                self._comms_lost_notified = True
                log.warning("Drive communications lost")
                if notif.on_comms_loss.value:
                    await self._notify("no communications with the drive")
            return
        if self._comms_lost_notified:
            self._comms_lost_notified = False
            log.info("Drive communications restored")
            if notif.on_comms_loss.value:
                await self._notify(
                    "communications with the drive restored", SEVERITY_INFO
                )

        # Trips: notify on the edge, and again if the code changes.
        trip = (status.tripped, status.trip_code if status.tripped else 0)
        if self._prev_trip is not None and trip != self._prev_trip and status.tripped:
            log.error(
                "Drive tripped: %s (code %s)", status.trip_description, status.trip_code
            )
            if notif.on_trip.value:
                await self._notify(
                    f"drive tripped: {status.trip_description} (code {status.trip_code})"
                )
        self._prev_trip = trip

        # Start / stop edges. The first poll only records state.
        if self._prev_running is not None and status.running != self._prev_running:
            if status.running:
                log.info("Motor running")
                if notif.on_start.value:
                    await self._notify("motor started", SEVERITY_INFO)
            else:
                log.info("Motor stopped")
                if notif.on_stop.value:
                    await self._notify("motor stopped", SEVERITY_INFO)
        self._prev_running = status.running

        if self.controller.start_failed:
            self.controller.start_failed = False
            log.error(
                "Drive did not start within %.0fs of the run command",
                self.controller.start_timeout_s,
            )
            await self._notify("drive did not start after a run command")
