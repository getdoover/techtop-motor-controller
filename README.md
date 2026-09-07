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

## Drive setup (keypad, one-off)

The drive rejects control-word writes unless it is in Modbus control mode, and
parameter writes over Modbus are refused on the TTA-3 firmware, so this has to
be done on the keypad:

| Parameter | Value | Why |
|---|---|---|
| P-14 | 101 | Unlock the extended menu |
| P-12 | 3 | Modbus control, drive's own ramps |
| P-36 idx 1 | 1 | Modbus address (match *Modbus Unit ID*) |
| P-36 idx 2 | 115.2 | Baud rate (match *Serial Baud*); 8 data bits, no parity, 1 stop bit are fixed |
| P-36 idx 3 | r 3000 recommended | Ramp-stop if no Modbus telegram for 3 s. Only meaningful with *Poll Interval* well under it. |
| P-01 .. P-04 | as required | Max/min frequency, accel, decel |
| P-07 .. P-10 | motor nameplate | Volts, amps, Hz, rpm |

Leave P-31 at its default (1). With P-12 = 3 the keypad's own Start/Stop keys
are ignored and the terminals only supply the enable.

Until P-12 = 3 the app still monitors the drive and shows a *Drive not in
Modbus control* warning; the control buttons stay hidden.

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

## Modbus notes

- The drive only has holding registers; documented register *N* is address
  *N-1* on the wire (pydoover `register_type=4`).
- The drive answers single-register writes (FC06). Multi-register writes to
  the control block were refused on the bench, so the app writes the setpoint
  and control word as two single writes, setpoint first.
- Parameters read back at register 128 + P-xx in the drive's internal formats
  (P-01 reads 3000 for 50.0 Hz). Writes to them are refused, hence the keypad
  setup above.
- Bit 15 of status word 2 toggles every second on this firmware; it is ignored.

## Testing without a motor

The drive runs with nothing on U/V/W: it reports Running, ramps the output
frequency and shows roughly zero current, which is enough to exercise every
state transition, the RPC surface and the UI. Current, power, torque and any
motor-related trips (overload, output fault) can only be checked with a motor
fitted.
