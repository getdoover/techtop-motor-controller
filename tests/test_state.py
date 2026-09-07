"""Sequencing tests for MotorController against canned drive statuses."""

import pytest

from techtop_motor_controller.app_state import (
    DISCONNECTED,
    NOT_READY,
    READY,
    RESETTING,
    RUNNING,
    STARTING,
    STOPPING,
    TRIPPED,
    MotorController,
)
from techtop_motor_controller.drive import CW_COAST_STOP, CW_FAST_STOP, CW_RESET, CW_RUN, DriveStatus


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def status(*, ready=True, running=False, tripped=False, trip_code=0, di1=True, standby=False):
    return DriveStatus(
        contactable=True,
        ready=ready,
        running=running,
        tripped=tripped,
        trip_code=trip_code,
        trip_description="Under voltage on DC bus" if tripped else "",
        di1=di1,
        standby=standby,
    )


OFFLINE = DriveStatus(contactable=False)


def make(clock=None, **kw):
    kw.setdefault("start_timeout_s", 10)
    kw.setdefault("stop_timeout_s", 30)
    return MotorController(clock=clock or Clock(), **kw)


@pytest.mark.asyncio
async def test_boot_sequence_to_ready():
    c = make()
    assert c.state == DISCONNECTED
    assert await c.spin(status(ready=False, di1=False), control_allowed=True) == NOT_READY
    assert await c.spin(status(), control_allowed=True) == READY
    assert c.control_word() == 0
    assert c.can_start and not c.can_stop and not c.can_reset


@pytest.mark.asyncio
async def test_start_then_stop_sequence():
    c = make()
    await c.spin(status(), control_allowed=True)
    c.setpoint_hz = 30
    c.request_start()
    assert await c.spin(status(), control_allowed=True) == STARTING
    assert c.control_word() == CW_RUN
    assert c.signed_setpoint_hz() == 30
    # drive picks up the run bit
    assert await c.spin(status(running=True), control_allowed=True) == RUNNING
    assert c.control_word() == CW_RUN
    c.request_stop()
    assert await c.spin(status(running=True), control_allowed=True) == STOPPING
    assert c.control_word() == 0  # ramp stop = drop the run bit
    assert await c.spin(status(running=False), control_allowed=True) == READY
    assert c.control_word() == 0


@pytest.mark.asyncio
async def test_fast_and_coast_stop_words():
    c = make()
    await c.spin(status(), control_allowed=True)
    c.request_start()
    await c.spin(status(), control_allowed=True)
    await c.spin(status(running=True), control_allowed=True)
    c.request_stop("fast")
    await c.spin(status(running=True), control_allowed=True)
    assert c.control_word() == CW_FAST_STOP
    c.stop_mode = "coast"
    assert c.control_word() == CW_COAST_STOP
    with pytest.raises(ValueError):
        c.request_stop("sideways")


@pytest.mark.asyncio
async def test_start_not_issued_without_control():
    c = make()
    await c.spin(status(), control_allowed=False)
    c.request_start()
    assert await c.spin(status(), control_allowed=False) == READY
    assert c.control_word() == 0


@pytest.mark.asyncio
async def test_start_timeout_returns_to_ready_and_flags_failure():
    clock = Clock()
    c = make(clock)
    await c.spin(status(), control_allowed=True)
    c.request_start()
    await c.spin(status(), control_allowed=True)
    assert c.state == STARTING
    clock.advance(11)
    assert await c.spin(status(), control_allowed=True) == READY
    assert c.start_failed
    assert not c.run_requested
    assert c.control_word() == 0


@pytest.mark.asyncio
async def test_trip_clears_run_request_and_reset_never_restarts():
    c = make()
    await c.spin(status(), control_allowed=True)
    c.request_start()
    await c.spin(status(), control_allowed=True)
    await c.spin(status(running=True), control_allowed=True)
    assert c.state == RUNNING and c.run_requested

    assert await c.spin(status(tripped=True, trip_code=7, ready=False), control_allowed=True) == TRIPPED
    assert not c.run_requested
    assert c.control_word() == 0
    assert c.can_reset

    c.request_reset()
    assert await c.spin(status(tripped=True, trip_code=7, ready=False), control_allowed=True) == RESETTING
    assert c.control_word() == CW_RESET
    # trip clears -> not_ready (enable evaluated next), run bit stays clear
    assert await c.spin(status(ready=True), control_allowed=True) == READY
    assert c.control_word() == 0
    assert not c.run_requested


@pytest.mark.asyncio
async def test_reset_timeout_falls_back_to_tripped():
    clock = Clock()
    c = make(clock)
    await c.spin(status(tripped=True, ready=False), control_allowed=True)
    assert c.state == TRIPPED
    c.request_reset()
    await c.spin(status(tripped=True, ready=False), control_allowed=True)
    assert c.state == RESETTING
    clock.advance(6)
    assert await c.spin(status(tripped=True, ready=False), control_allowed=True) == TRIPPED


@pytest.mark.asyncio
async def test_reset_request_ignored_without_control():
    c = make()
    await c.spin(status(tripped=True, ready=False), control_allowed=False)
    c.request_reset()
    assert await c.spin(status(tripped=True, ready=False), control_allowed=False) == TRIPPED


@pytest.mark.asyncio
async def test_trip_cleared_at_keypad():
    c = make()
    await c.spin(status(tripped=True, ready=False), control_allowed=True)
    assert await c.spin(status(), control_allowed=True) == READY


@pytest.mark.asyncio
async def test_adopts_a_drive_already_running():
    c = make()
    assert await c.spin(status(running=True), control_allowed=True) == RUNNING
    assert c.run_requested
    assert c.control_word() == CW_RUN


@pytest.mark.asyncio
async def test_external_stop_while_running():
    c = make()
    await c.spin(status(running=True), control_allowed=True)
    assert await c.spin(status(running=False, ready=False, di1=False), control_allowed=True) == NOT_READY
    assert not c.run_requested


@pytest.mark.asyncio
async def test_enable_dropped_while_starting():
    c = make()
    await c.spin(status(), control_allowed=True)
    c.request_start()
    await c.spin(status(), control_allowed=True)
    assert await c.spin(status(ready=False, di1=False), control_allowed=True) == NOT_READY


@pytest.mark.asyncio
async def test_comms_loss_clears_run_request_and_reconnect_adopts_reality():
    c = make()
    await c.spin(status(running=True), control_allowed=True)
    assert await c.spin(OFFLINE, control_allowed=True) == DISCONNECTED
    assert not c.run_requested
    assert await c.spin(None, control_allowed=True) == DISCONNECTED
    # back, and the drive is idle: we do not restart it
    assert await c.spin(status(), control_allowed=True) == READY
    assert c.control_word() == 0
    # back, and the drive kept running: adopt it
    assert await c.spin(status(running=True), control_allowed=True) == RUNNING


@pytest.mark.asyncio
async def test_stop_overdue_is_flagged_once():
    clock = Clock()
    c = make(clock)
    await c.spin(status(running=True), control_allowed=True)
    c.request_stop()
    await c.spin(status(running=True), control_allowed=True)
    assert c.state == STOPPING
    clock.advance(31)
    await c.spin(status(running=True), control_allowed=True)
    assert c.stop_overdue and c.state == STOPPING


@pytest.mark.asyncio
async def test_state_change_callback():
    seen = []

    async def on_change(old, new):
        seen.append((old, new))

    c = MotorController(clock=Clock(), on_state_change=on_change)
    await c.spin(status(), control_allowed=True)
    assert seen == [(DISCONNECTED, NOT_READY), (NOT_READY, READY)]


def test_signed_setpoint_direction():
    c = make()
    c.setpoint_hz = 20
    assert c.signed_setpoint_hz() == 20
    c.reverse = True
    assert c.signed_setpoint_hz() == -20
