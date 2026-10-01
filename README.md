# Techtop Motor Controller

Doover device app that monitors and controls a **Techtop TTA-3** variable
frequency drive over Modbus RTU from a Doovit. The TTA-3 is a rebadged
**Invertek Optidrive E3**, so this app follows the Invertek E3 register map and
works on either badge.

- Live drive state, output frequency, current, power, torque, DC bus, temperatures,
  run hours and energy
- Start / stop / speed setpoint / trip reset from the Doover UI
- The same commands over **RPC**, so other Doover apps (a pump controller, a
  scheduler, an HMI) can drive the motor through this app
- Commissioning RPCs: live diagnostics and reading / writing an allowlist of
  drive parameters with range checks and read-back
- A sequencing state machine layered over the drive's own Not Ready / Ready /
  Running / Tripped states, with timeouts and a trip that always clears the run
  request
- Optional hardware-enable output: a Doovit digital output wired to the drive's
  DI1 is held high while the app runs and dropped when the Doovit shuts down
- Notifications on trip, comms loss and (optionally) start/stop

## Wiring

**Mains:** L1, L2, L3 and earth only. The `L2/N` label is shared silkscreen
with the single-phase models; do not connect neutral.

**Modbus RTU** is on the drive's front RJ45 (not Ethernet). Cut one end off a
patch lead:

| Doovit | Drive RJ45 | T568B colour |
|---|---|---|
| RS485 A | pin 8, RS485+ | brown |
| RS485 B | pin 7, RS485- | white/brown |
| 0V | pin 3, 0V common | white/green |

A/B naming is not standardised between vendors: if the drive never answers,
swap the two data wires.

**Hardware enable (required for Modbus control):** the drive will not report
Ready, and will not run, unless terminal 2 (Digital Input 1) sees 8-30 V DC.
Either wire a Doovit digital output to terminal 2 with the Doovit 0V to
terminal 7 and set *Enable Output Pin* in the config, or permanently link
terminal 1 (+24 V) to terminal 2. Using a Doovit output is preferred: the app
drops it on shutdown, which stops the motor if the Doovit powers down.

The app only drives that output high once it has read P-12 = 3 from the drive
and control is enabled. In terminal mode (P-12 = 0) DI1 is the *run* command,
so asserting it there would start the motor, and an enabled drive also refuses
keypad edits to parameters such as P-12 (the display flashes with an `L`). If
you need to edit a locked parameter while the app is running, set *Control
Enabled* off and redeploy, or stop the app's container.

## Drive setup (keypad, one-off)

The drive rejects control-word writes unless it is in Modbus control mode, so
P-12 has to be set on the keypad first. The motor nameplate (P-07 .. P-10) can
be set on the keypad too, or left to the app's *Motor Rated* config fields
(see Configuration). P-36 can only be set on the keypad: the drive excludes all
three of its indices from Modbus access.

| Parameter | Value | Why |
|---|---|---|
| P-14 | 101 | Unlock the extended menu |
| P-12 | 3 | Modbus control, drive's own ramps |
| P-36 idx 1 | 1 | Modbus address (match *Modbus Unit ID*) |
| P-36 idx 2 | 115.2 | Baud rate (match *Serial Baud*); 8 data bits, no parity, 1 stop bit are fixed |
| P-36 idx 3 | t 3000 (factory default, kept) | Comms-loss watchdog: trip if no control-word write for 3 s while enabled. `r 3000` ramps to a stop instead of tripping. Keep *Poll Interval* well under it. |
| P-01 .. P-04 | as required | Max/min frequency, accel, decel |
| P-07 .. P-10 | motor nameplate | Volts, amps, Hz, rpm. Optional here if set in the app config |

Leave P-31 at its default (1). With P-12 = 3 the keypad's own Start/Stop keys
are ignored and the terminals only supply the enable.

Until P-12 = 3 the app still monitors the drive and shows a *Drive not in
Modbus control* warning; the control buttons stay hidden.

### Changing a parameter on the keypad

The app cannot change P-12 (or P-14, P-36, P-37, P-38): `write_parameter`
refuses them on purpose, so nobody can move a drive between terminal and Modbus
control remotely. They are set on the drive:

1. Make sure the drive is **not enabled**: terminal 2 (DI1) off. If the app
   holds the enable output high, set *Control Enabled* off and redeploy, or
   stop the app's container. An enabled drive refuses the edit and the display
   flashes `L`.
2. Hold **Navigate** (the middle key) for about 2 s until a parameter number
   (`P-01` ...) shows.
3. Use **Up** / **Down** to reach the parameter, then press **Navigate** to
   show its value.
4. Use **Up** / **Down** to set the value, then press **Navigate** to store it.
5. Hold **Navigate** for about 2 s to return to the normal display.

P-01 .. P-14 are always visible; for P-15 and above, set P-14 = 101 first (see
the table above and `docs/optidrive-e3-reference.md`).

Within one *Parameter Refresh* (60 s by default) the app logs
`Drive control source (P-12): modbus` and the warning clears. Its once-a-minute
status line also shows the mode, e.g. `Drive running (controller running,
P-12 modbus): out 50.0 Hz, ...`.

When the keypad display shows a number that is not the frequency, it may be on
another readout: **Navigate** (a short press) steps through output frequency,
motor current (A), power (kW) and speed. A drive at 50 Hz under light load can
show `0.3` because it is on the kW readout.

## Doovit RS-485 port

The Doovit's RS-485 transceiver sits behind its IO microcontroller, which has
its own baud setting that must match the drive as well as the app's Modbus
config. On the Doovit:

```
dvt set_serial_params 115200 True True 8 N 1 300 50 True
```

(baud, RS485 mode, terminator, bits, parity, stop, timeout ms, read chunk ms,
invert A/B). The inversion flag has no effect; swap the wires instead.

## Configuration

| Key | Default | Meaning |
|---|---|---|
| `modbus_config` | serial, `/dev/ttyAMA0`, 115200 8N1 | pydoover Modbus bus. Use `tcp` for a serial-to-Ethernet gateway. |
| `modbus_unit_id` | 1 | Drive address, P-36 index 1 |
| `enable_output_pin` | none | Doovit DO wired to drive terminal 2 |
| `motor_rated_voltage_v` / `motor_rated_current_a` / `motor_rated_frequency_hz` / `motor_rated_speed_rpm` | blank | Motor nameplate, written to P-07 .. P-10. Blank = keep the drive's value |
| `control_enabled` | true | false = monitor only, nothing is written to the drive |
| `max_frequency_hz` / `min_frequency_hz` | 50 / 0 | Setpoint limits (also capped by the drive's P-01) |
| `default_frequency_hz` | 50 | Setpoint for a start that names no frequency |
| `stop_mode` | ramp | `ramp` (P-04), `fast` (P-24) or `coast` |
| `start_timeout_s` / `stop_timeout_s` | 10 / 60 | How long to wait for the drive to follow |
| `poll_interval_s` | 1.0 | Poll and control-word refresh period |
| `parameter_refresh_s` | 60 | Re-read P-12, limits and motor rating |
| `comms_loss_timeout_s` | 30 | Silence before *disconnected* |
| `notifications` | trip, comms loss | Which events notify |

## State machine

The drive reports its own state in status word 2 (register 2001); the app's
`MotorController` sequences commands against it:

```
disconnected ──► not_ready ──► ready ──► starting ──► running ──► stopping ──► ready
                    ▲            ▲                                              
                    └── tripped ◄┴────────── (any state, on the drive's trip bit)
                          │   ▲
                       resetting ┘ (reset bit pulsed; timeout falls back to tripped)
```

Rules worth knowing:

- A trip clears the run request. Resetting a trip never restarts the motor.
- A start that the drive does not acknowledge within *Start Timeout* is
  abandoned and reported.
- A drive found already running (after a restart, or started elsewhere) is
  adopted as running, not stopped.
- Comms loss clears the run request. On reconnect the app mirrors whatever the
  drive is doing; it does not restart a motor by itself.
- While in Modbus mode the control word and setpoint are rewritten every poll,
  which keeps the drive's Modbus watchdog (P-36 index 3) fed.

## Tags

`comms_active`, `controller_state`, `drive_state`, `control_source`,
`modbus_control`, `running`, `ready`, `tripped`, `trip_code`, `trip_description`,
`enable_present`, `at_speed`, `mains_loss`, `overload`, `direction`,
`frequency_setpoint_hz`, `output_frequency_hz`, `motor_current_a`,
`motor_power_kw`, `torque_pct`, `output_voltage_v`, `dc_bus_voltage_v`,
`heatsink_temp_c`, `internal_temp_c`, `energy_kwh`, `run_hours`, `di1`..`di4`,
`relay_closed`, `drive_max_frequency_hz`, `motor_rated_current_a`, `last_command`.

## RPC interface

Other apps on the same device call this app over pydoover RPC (the default
`dv-rpc` channel). `app_key` is this app's install key, e.g.
`techtop_motor_controller_1`.

```python
result = await self.rpc.call(
    "start", params={"frequency_hz": 30}, app_key="techtop_motor_controller_1"
)
```

| Method | Params | Effect |
|---|---|---|
| `start` | `frequency_hz` (optional), `direction` (`forward`/`reverse`, optional) | Set the setpoint and run |
| `stop` | `mode` (`ramp`/`fast`/`coast`, optional) | Stop |
| `set_frequency` | `frequency_hz`, `direction` (optional) | Change the setpoint; takes effect immediately if running |
| `reset_fault` | – | Pulse the drive's reset bit |
| `get_status` | – | Current status |

Every method returns the status dict:

```json
{
  "comms_active": true, "controller_state": "running", "drive_state": "running",
  "control_source": "modbus", "modbus_control": true, "control_enabled": true,
  "running": true, "ready": true, "tripped": false, "trip_code": null,
  "trip_description": null, "enable_present": true,
  "requested_frequency_hz": 30.0, "drive_setpoint_hz": 30.0,
  "output_frequency_hz": 30.0, "motor_current_a": 1.9, "motor_power_kw": 0.61,
  "torque_pct": 48.2, "dc_bus_voltage_v": 590.0, "direction": "forward"
}
```

Failures raise `RPCError` with one of: `CONTROL_DISABLED`, `NOT_CONNECTED`,
`NOT_MODBUS_CONTROL`, `NOT_READY`, `TRIPPED`, `NOT_TRIPPED`,
`INVALID_FREQUENCY`, `INVALID_DIRECTION`, `INVALID_STOP_MODE`.

## Commissioning RPCs

The commissioning panel calls three more methods on the same `dv-rpc`
channel. They work in monitor-only mode too, except that writes are refused.

### `get_diagnostics` `{}`

One fresh read of the status block, setpoint and meters per call; nothing is
polled in the background for it.

```json
{
  "output_hz": 25.0, "output_current_a": 0.8, "motor_rpm": 695,
  "dc_bus_v": 592.0, "heatsink_c": 30.0, "drive_state": "running",
  "trip_code": null, "trip_description": null, "run_hours": 10.5,
  "recent_trips": null, "comms_ok": true
}
```

- `drive_state`: `disconnected`, `not_ready`, `ready`, `running`, `standby`
  or `tripped`.
- `trip_code` / `trip_description` are null unless the drive is tripped.
- `motor_rpm` is an estimate (output Hz x P-10 / P-09), as the drive's display
  shows it. It is null while P-10 is 0 or unknown: the E3 does not report shaft
  speed over Modbus.
- `recent_trips` is always null: the E3 keeps its trip log (P00-13) on the
  keypad only; it is not in the Modbus register map.
- Without comms, `comms_ok` is false, `drive_state` is `disconnected` and every
  other value is null.

### `read_parameters` `{}`

```json
{"parameters": [
  {"id": "P-09", "name": "Motor rated frequency", "value": 50, "units": "Hz",
   "min": 10, "max": 500, "step": 1, "writable": true, "stop_required": true,
   "description": "Motor nameplate frequency. Changing it resets P-10 and the preset speeds on the drive."}
]}
```

`value` is null if the drive did not answer. P-01's `min` is the drive's
current P-02 and P-02's `max` is its current P-01. A null `max` means the
drive's own rating (P-08).

### `write_parameter` `{"parameter": "P-09", "value": 50}`

Returns the read-back value, `{"parameter": "P-09", "value": 50}`, or an
`RPCError`:

| Code | When |
|---|---|
| `NOT_ALLOWED` | Not on the allowlist, never writable (P-12, P-14, P-36, P-37, P-38), set by the app config (see below), or control disabled |
| `OUT_OF_RANGE` | Not a number, outside `min`..`max`, not a multiple of `step`, or refused by the drive (e.g. P-08 above the drive's rating) |
| `DRIVE_RUNNING` | `stop_required` and the drive reports Running |
| `READBACK_MISMATCH` | The drive accepted the write but reads back a different value |
| `COMMS_ERROR` | No reply from the drive before, during or after the write |

Every write reads the drive's state first, range-checks and scales the value,
writes it with FC06, reads it back and compares the raw values. It is logged
with the value, the previous value and the RPC actor, and recorded in
`last_command`. Parameters are re-read on the next poll.

### Parameters

Ranges and units are from the E3 IP20 User Guide (V1.05, section 6.1).

| Id | Name | Units | Range | Step | Writable | Stop required |
|---|---|---|---|---|---|---|
| P-01 | Maximum frequency | Hz | P-02 .. 500 | 0.1 | yes | no |
| P-02 | Minimum frequency | Hz | 0 .. P-01 | 0.1 | yes | no |
| P-03 | Acceleration time | s | 0 .. 600 | 0.01 | yes | no |
| P-04 | Deceleration time | s | 0 .. 600 | 0.01 | yes | no |
| P-05 | Stopping mode | – | 0 .. 4 | 1 | yes | yes |
| P-07 | Motor rated voltage | V | 0 .. 500 | 1 | yes, unless set in config | yes |
| P-08 | Motor rated current | A | 0 .. drive rating | 0.1 | yes, unless set in config | yes |
| P-09 | Motor rated frequency | Hz | 10 .. 500 | 1 | yes, unless set in config | yes |
| P-10 | Motor rated speed | rpm | 0 .. 30000 | 1 | yes, unless set in config | yes |
| P-12 | Control source | – | 0 .. 9 | 1 | never | – |
| P-24 | Fast stop ramp time | s | 0 .. 600 | 0.01 | yes | no |
| P-36 | Modbus address | – | 0 .. 63 | 1 | never (keypad only) | – |

- The user guide gives no per-parameter stop-only rules. The app requires the
  motor stopped for the motor data (P-07..P-10) and the stopping mode (P-05),
  and allows ramps, limits and P-24 while running.
- P-24 is taken to use the same 0.01 s internal format as P-03 / P-04; the
  other scalings are bench-verified.
- P-12, the P-36 comms settings (address, baud, watchdog) and the keypad access
  parameters (P-14, P-37, P-38) are never written: a wrong value there cuts the
  app off from the drive or takes it out of Modbus control. The E3 has no
  factory-reset parameter (it is a keypad key combination).
- P-36 is reported as the drive address, with the baud rate and comms-loss
  setting in its description.

### Nameplate config and the panel

The motor nameplate fields in the app config (`motor_rated_*`) are written to
P-07..P-10 at startup and on every parameter refresh whenever the drive reads
back different. A panel write to one of those parameters would be overwritten
within a minute, so the app treats the config as the owner:

- A nameplate parameter whose config field is **set** is reported with
  `writable: false` and the description `Set in the app config (nameplate)`,
  and `write_parameter` refuses it with `NOT_ALLOWED` and that reason. Change
  it in the app config instead.
- A nameplate parameter whose config field is **blank** is not managed by the
  app and is writable from the panel like any other.

This is per field: setting only *Motor Rated Current* locks P-08 and leaves
P-07, P-09 and P-10 writable. To commission the nameplate from the panel,
leave the four `motor_rated_*` fields blank.

## Modbus notes

- The drive only has holding registers; documented register *N* is address
  *N-1* on the wire (pydoover `register_type=4`).
- Invertek documents only two function codes for the E3: FC03 (Read Holding
  Registers) and FC06 (Write Single Holding Register). See
  [docs/optidrive-e3-reference.md](docs/optidrive-e3-reference.md).
- The control word and setpoint are written as two one-value writes through
  pydoover's `write_registers` (FC16), setpoint first. This firmware accepts a
  one-value FC16 there (undocumented), but refuses a multi-register FC16 to
  that block.
- Parameters read back at register 128 + P-xx in the drive's internal formats
  (P-01 reads 3000 for 50.0 Hz). The drive refuses FC16 on these registers
  (IllegalFunction) but accepts FC06, so parameter writes use pydoover's
  `write_single_register`. That needs a modbus interface that implements the
  `writeSingleRegister` RPC.
- The drive caps each parameter at its own limits and answers an out of range
  value with IllegalValue. P-08 cannot exceed the drive's rated current (2.2 A
  on the 0.37 kW, 400 V frame).
- Nameplate values are written only while control is enabled and the motor is
  stopped, at startup and on each parameter refresh, and only when the drive
  reads back a different value. A value the drive refuses is logged and not
  retried until the app restarts.
- P-36 (register 164) reads back as one packed word: bits 0-7 address,
  bits 8-11 baud rate, bits 12-15 comms-loss setting. It is read-only over
  Modbus. The bench drive reads `0x4601`: address 1, 115.2 kbps, `t 3000`.
- With `t 3000`, restarting the app or the modbus interface while the drive is
  enabled can let the watchdog expire, which trips the drive. Clear it with
  *Reset Trip*; it does not restart the motor.
- Bit 15 of status word 2 toggles every second on this firmware; it is ignored.

## Testing without a motor

The drive runs with nothing on U/V/W: it reports Running, ramps the output
frequency and shows roughly zero current, which is enough to exercise every
state transition, the RPC surface and the UI. Current, power, torque and any
motor-related trips (overload, output fault) can only be checked with a motor
fitted.
