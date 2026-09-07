# Techtop Motor Controller -- Development Guide

## Repository Structure

```
src/techtop_motor_controller/
  __init__.py       <-- Docker device entry point (run_app)
  application.py    <-- Poll/control cycle, UI handlers, RPC surface, tags, notifications
  app_config.py     <-- Config schema (Modbus bus, enable pin, limits, notifications)
  app_tags.py       <-- Persisted state tags (also drive the UI's conditional visibility)
  app_ui.py         <-- UI elements
  app_state.py      <-- MotorController: sequencing state machine over the drive's state
  drive.py          <-- TechtopDrive: E3 register map, decoding, single-register writes
simulators/
  docker-compose.yml  <-- Hand-run the app on a Doovit (APP_KEY + CONFIG_FP)
  app_config.json     <-- Config for that hand-run
tests/                <-- pytest suite (driver decode, state machine, app plumbing)
```

## Architecture

```
UI button / RPC call ─► Application.command_*() ─► MotorController (request)
                                                          │
             main_loop (every poll_interval_s)            ▼
   TechtopDrive.read_status() ──► MotorController.spin(status) ──► control_word()
              ▲                                                         │
              └──────────── modbus_iface (gRPC) ◄── TechtopDrive.write_command() ◄─┘
```

- **`drive.py`** is pydoover-free. It knows the E3 register map (status block
  2001-2016, control word 1, setpoint 2, meters 32-35, direct parameters at
  128 + P-xx) and only ever issues single-register writes, because that is what
  the drive accepts on the bench (see README "Modbus notes").
- **`app_state.py`** wraps the drive's own state machine. It never touches
  Modbus; it consumes `DriveStatus` snapshots and answers with the control word
  to write. Timeouts use an injectable clock so the tests are deterministic.
- **`application.py`** owns the cycle: read, spin, write (only when the drive
  is in Modbus control mode and control is enabled), publish tags, edge-detect
  notifications. UI and RPC commands share `command_start/stop/set_frequency/
  reset`, which validate, set a request on the controller and run one cycle
  immediately so the command is on the wire before the handler returns.

Visibility of buttons and warnings is bound to `hide_*` tags rather than set on
the elements, because the UI schema is published once at setup.

## Getting Started

```bash
uv sync
uv run pytest tests/
```

## Running by hand on a Doovit

```bash
rsync -a --exclude .venv --exclude .git ./ doovit@doovit-<serial>.local:~/techtop-motor-controller/
ssh doovit@doovit-<serial>.local
cd techtop-motor-controller && docker compose -f simulators/docker-compose.yml up --build
```

Edit `simulators/app_config.json` first (unit id, enable pin, limits). The Doovit's
RS-485 bridge must be set to the drive's baud rate; see the README.

## Regenerating doover_config.json

The committed `doover_config.json` holds the exported `config_schema` and
`ui_schema`; CI fails if they drift from the Python source.

```bash
uv run export-config
uv run export-ui
```

## Publishing

Pushes to `main` run `.github/workflows/doover-app.yml`, which calls the shared
`getdoover/workflows` app workflow: lint, tests, schema validation, image build,
publish and release to Doover via GitHub OIDC trusted publishing. Pull requests
release an alpha.
