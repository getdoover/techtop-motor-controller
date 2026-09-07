"""Sequencing state machine for the drive.

The drive already has a state machine of its own -- Not Ready / Ready /
Running / Tripped, reported in its status word -- and it is authoritative. This
controller does not try to *be* that machine; it wraps it, so that:

* every command (start, stop, reset) is sequenced against what the drive
  actually reports, with a timeout when the drive does not follow;
* a trip always clears the run request, so a reset can never restart the motor
  by itself;
* a drive found already running (after a container restart, or started from
  elsewhere) is adopted as running rather than stopped or fought over.

The controller is pure logic: it reads :class:`DriveStatus` snapshots and
answers with the control word the application should write. It never touches
Modbus itself, which is what makes it unit-testable with canned statuses.

Timeouts are checked against an injectable clock inside :meth:`evaluate`
rather than with the ``transitions`` timeout feature, so tests stay
deterministic.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Awaitable, Callable
from typing import TYPE_CHECKING

from pydoover.state import StateMachine

from .drive import DriveStatus, compose_control_word

log = logging.getLogger(__name__)

STOP_MODES = ("ramp", "fast", "coast")

DISCONNECTED = "disconnected"
NOT_READY = "not_ready"
READY = "ready"
STARTING = "starting"
RUNNING = "running"
STOPPING = "stopping"
TRIPPED = "tripped"
RESETTING = "resetting"

STATES = [
    DISCONNECTED,
    NOT_READY,
    READY,
    STARTING,
    RUNNING,
    STOPPING,
    TRIPPED,
    RESETTING,
]


class MotorController:
    """Sequences start / stop / reset against the drive's reported state."""

    states = STATES

    transitions = [
        # Comms
        {"trigger": "comms_lost", "source": "*", "dest": DISCONNECTED},
        {"trigger": "comms_restored", "source": DISCONNECTED, "dest": NOT_READY},
        # Ready / not ready
        {
            "trigger": "became_ready",
            "source": [NOT_READY, STOPPING, STARTING],
            "dest": READY,
        },
        {
            "trigger": "lost_ready",
            "source": [READY, STARTING, STOPPING],
            "dest": NOT_READY,
        },
        # Start sequence
        {"trigger": "start_command", "source": READY, "dest": STARTING},
        {"trigger": "started", "source": STARTING, "dest": RUNNING},
        {"trigger": "start_timed_out", "source": STARTING, "dest": READY},
        {"trigger": "start_cancelled", "source": STARTING, "dest": READY},
        # A drive already running (restart, keypad, terminals) is adopted.
        {"trigger": "adopt_running", "source": [NOT_READY, READY], "dest": RUNNING},
        # Stop sequence
        {"trigger": "stop_command", "source": RUNNING, "dest": STOPPING},
        {"trigger": "stopped", "source": STOPPING, "dest": READY},
        {"trigger": "stopped_not_ready", "source": STOPPING, "dest": NOT_READY},
        {"trigger": "stopped_externally", "source": RUNNING, "dest": READY},
        {
            "trigger": "stopped_externally_not_ready",
            "source": RUNNING,
            "dest": NOT_READY,
        },
        # Trips
        {"trigger": "trip_detected", "source": "*", "dest": TRIPPED},
        {"trigger": "trip_cleared", "source": TRIPPED, "dest": NOT_READY},
        {"trigger": "reset_command", "source": TRIPPED, "dest": RESETTING},
        {"trigger": "reset_succeeded", "source": RESETTING, "dest": NOT_READY},
        {"trigger": "reset_failed", "source": RESETTING, "dest": TRIPPED},
    ]

    def __init__(
        self,
        *,
        start_timeout_s: float = 10.0,
        stop_timeout_s: float = 60.0,
        reset_timeout_s: float = 5.0,
        default_stop_mode: str = "ramp",
        clock: Callable[[], float] = time.monotonic,
        on_state_change: Callable[[str, str], Awaitable[None]] | None = None,
    ):
        self.start_timeout_s = start_timeout_s
        self.stop_timeout_s = stop_timeout_s
        self.reset_timeout_s = reset_timeout_s
        self.default_stop_mode = default_stop_mode
        self._clock = clock
        self._on_state_change = on_state_change

        # What the operator / peer app has asked for.
        self.run_requested = False
        self.reset_requested = False
        self.stop_mode = default_stop_mode
        self.setpoint_hz = 0.0
        self.reverse = False

        # Bookkeeping
        self.state_entered_at = clock()
        self.previous_state = DISCONNECTED
        self.start_failed = False  # set when a start timed out; cleared on next start
        self.stop_overdue = False  # set once when a stop exceeds stop_timeout_s
        self.last_status: DriveStatus = DriveStatus()

        self.state_machine = StateMachine(
            states=self.states,
            transitions=self.transitions,
            model=self,
            initial=DISCONNECTED,
            queued=True,
            after_state_change="_after_state_change",
            send_event=False,
        )

    # -- requests (from UI / RPC) --------------------------------------------

    def request_start(self) -> None:
        self.run_requested = True
        self.start_failed = False

    def request_stop(self, mode: str | None = None) -> None:
        if mode is not None:
            if mode not in STOP_MODES:
                raise ValueError(f"stop mode must be one of {STOP_MODES}")
            self.stop_mode = mode
        else:
            self.stop_mode = self.default_stop_mode
        self.run_requested = False

    def request_reset(self) -> None:
        self.reset_requested = True

    # -- outputs -------------------------------------------------------------

    def control_word(self) -> int:
        """The control word to write for the current state."""
        if self.state in (STARTING, RUNNING):
            return compose_control_word(run=True)
        if self.state == STOPPING:
            return compose_control_word(
                fast_stop=self.stop_mode == "fast",
                coast_stop=self.stop_mode == "coast",
            )
        if self.state == RESETTING:
            return compose_control_word(reset=True)
        return compose_control_word()

    def signed_setpoint_hz(self) -> float:
        return -abs(self.setpoint_hz) if self.reverse else abs(self.setpoint_hz)

    @property
    def seconds_in_state(self) -> float:
        return self._clock() - self.state_entered_at

    @property
    def can_start(self) -> bool:
        return self.state == READY

    @property
    def can_stop(self) -> bool:
        return self.state in (STARTING, RUNNING)

    @property
    def can_reset(self) -> bool:
        return self.state == TRIPPED

    # -- evaluation ----------------------------------------------------------

    async def spin(
        self,
        status: DriveStatus | None,
        *,
        control_allowed: bool,
        max_iterations: int = 6,
    ) -> str:
        """Evaluate repeatedly until the state settles; returns the final state."""
        for _ in range(max_iterations):
            before = self.state
            await self.evaluate(status, control_allowed=control_allowed)
            if self.state == before:
                break
        return self.state

    async def evaluate(
        self, status: DriveStatus | None, *, control_allowed: bool
    ) -> None:
        """One evaluation step against a drive status snapshot.

        ``control_allowed`` is whether this app may command the drive right now
        (control enabled in config and the drive in Modbus control mode).
        Without it the controller only mirrors the drive.
        """
        if status is None or not status.contactable:
            if self.state != DISCONNECTED:
                await self.comms_lost()
            return

        self.last_status = status
        state = self.state

        if state == DISCONNECTED:
            await self.comms_restored()
            return

        # A trip overrides everything.
        if status.tripped:
            if state == RESETTING:
                if self.seconds_in_state > self.reset_timeout_s:
                    await self.reset_failed()
                return
            if state != TRIPPED:
                await self.trip_detected()
                return
            if self.reset_requested:
                self.reset_requested = False
                if control_allowed:
                    await self.reset_command()
                return
            return

        # Not tripped from here on.
        if state == TRIPPED:
            await self.trip_cleared()
            return
        if state == RESETTING:
            await self.reset_succeeded()
            return

        self.reset_requested = False

        if state == NOT_READY:
            if status.running:
                await self.adopt_running()
            elif status.ready:
                await self.became_ready()
            return

        if state == READY:
            if status.running:
                await self.adopt_running()
            elif not status.ready:
                await self.lost_ready()
            elif self.run_requested and control_allowed:
                await self.start_command()
            return

        if state == STARTING:
            if status.running:
                await self.started()
            elif not self.run_requested:
                await self.start_cancelled()
            elif not status.ready:
                await self.lost_ready()
            elif self.seconds_in_state > self.start_timeout_s:
                await self.start_timed_out()
            return

        if state == RUNNING:
            if not status.running:
                # Stopped by something other than us: keypad, terminals, sleep.
                self.run_requested = False
                if status.ready:
                    await self.stopped_externally()
                else:
                    await self.stopped_externally_not_ready()
            elif not self.run_requested:
                await self.stop_command()
            return

        if state == STOPPING:
            if not status.running:
                if status.ready:
                    await self.stopped()
                else:
                    await self.stopped_not_ready()
            elif not self.stop_overdue and self.seconds_in_state > self.stop_timeout_s:
                self.stop_overdue = True
                log.warning(
                    "Drive still running %.0fs after a %s stop was commanded",
                    self.seconds_in_state,
                    self.stop_mode,
                )
            return

    # -- callbacks -----------------------------------------------------------

    async def _after_state_change(self, *args, **kwargs) -> None:
        # `transitions` keeps model.state current before this fires.
        new_state = self.state
        old_state = self.previous_state
        self.state_entered_at = self._clock()
        self.previous_state = new_state
        if old_state != new_state:
            log.info("Controller state: %s -> %s", old_state, new_state)
            if self._on_state_change is not None:
                await self._on_state_change(old_state, new_state)

    async def on_enter_tripped(self) -> None:
        # A reset must never restart the motor on its own.
        self.run_requested = False

    async def on_enter_disconnected(self) -> None:
        # Do not auto-restart after a comms outage: the drive has either kept
        # running (adopted on reconnect) or tripped on its watchdog.
        self.run_requested = False

    async def on_enter_starting(self) -> None:
        self.start_failed = False
        self.stop_overdue = False

    async def on_enter_stopping(self) -> None:
        self.stop_overdue = False

    async def on_enter_running(self) -> None:
        # Adopted or started: either way the run request now mirrors reality.
        self.run_requested = True

    async def on_enter_ready(self) -> None:
        if self.previous_state == STARTING and self.run_requested:
            # Arrived here from start_timed_out.
            self.start_failed = True
            self.run_requested = False

    if TYPE_CHECKING:
        # Trigger methods are generated by `transitions` at runtime; declaring
        # them for real would stop it binding them (model override policy).
        state: str

        async def comms_lost(self) -> bool: ...
        async def comms_restored(self) -> bool: ...
        async def became_ready(self) -> bool: ...
        async def lost_ready(self) -> bool: ...
        async def start_command(self) -> bool: ...
        async def started(self) -> bool: ...
        async def start_timed_out(self) -> bool: ...
        async def start_cancelled(self) -> bool: ...
        async def adopt_running(self) -> bool: ...
        async def stop_command(self) -> bool: ...
        async def stopped(self) -> bool: ...
        async def stopped_not_ready(self) -> bool: ...
        async def stopped_externally(self) -> bool: ...
        async def stopped_externally_not_ready(self) -> bool: ...
        async def trip_detected(self) -> bool: ...
        async def trip_cleared(self) -> bool: ...
        async def reset_command(self) -> bool: ...
        async def reset_succeeded(self) -> bool: ...
        async def reset_failed(self) -> bool: ...
