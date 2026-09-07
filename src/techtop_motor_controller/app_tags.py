from pydoover.tags import AnyChange, Boolean, Delta, Number, String, Tags


class TechtopMotorControllerTags(Tags):
    app_display_name = String(default="Motor Controller")

    # --- Connection / mode -------------------------------------------------
    comms_active = Boolean(default=False, log_on=AnyChange())
    # The app's own sequencing state (see app_state.py).
    controller_state = String(default="disconnected", log_on=AnyChange())
    # The drive's own state, straight from its status word.
    drive_state = String(default="disconnected", log_on=AnyChange())
    # Where the drive takes its commands from (P-12): terminal / keypad / modbus / ...
    control_source = String(default=None, log_on=AnyChange())
    modbus_control = Boolean(default=False, log_on=AnyChange())

    # --- Drive state flags --------------------------------------------------
    running = Boolean(default=False, log_on=AnyChange())
    ready = Boolean(default=False)
    tripped = Boolean(default=False, log_on=AnyChange())
    trip_code = Number(default=None)
    trip_description = String(default=None)
    enable_present = Boolean(default=False, log_on=AnyChange())
    at_speed = Boolean(default=False)
    mains_loss = Boolean(default=False, log_on=AnyChange())
    overload = Boolean(default=False, log_on=AnyChange())
    direction = String(default="forward")

    # --- Operating values ---------------------------------------------------
    # live=True streams these while someone has the app open. Delta thresholds
    # are coarse: they log the step from idle to running, not every ripple.
    frequency_setpoint_hz = Number(default=None, live=True, log_on=Delta(amount=1))
    output_frequency_hz = Number(default=None, live=True, log_on=Delta(amount=2))
    motor_current_a = Number(default=None, live=True, log_on=Delta(amount=0.5))
    motor_power_kw = Number(default=None, live=True, log_on=Delta(amount=0.2))
    torque_pct = Number(default=None, live=True)
    output_voltage_v = Number(default=None, live=True)
    dc_bus_voltage_v = Number(default=None, live=True)
    heatsink_temp_c = Number(default=None)
    internal_temp_c = Number(default=None)
    energy_kwh = Number(default=None)
    run_hours = Number(default=None)

    # --- Drive IO -----------------------------------------------------------
    di1 = Boolean(default=False)
    di2 = Boolean(default=False)
    di3 = Boolean(default=False)
    di4 = Boolean(default=False)
    relay_closed = Boolean(default=False)

    # --- Drive parameters (read from the drive) -----------------------------
    drive_max_frequency_hz = Number(default=None)
    motor_rated_current_a = Number(default=None)

    # Last command this app issued, for the activity trail.
    last_command = String(default=None, log_on=AnyChange())

    # --- UI visibility (resolved by the site on every render) ---------------
    hide_no_comms_warning = Boolean(default=True)
    hide_trip_warning = Boolean(default=True)
    hide_not_enabled_warning = Boolean(default=True)
    hide_not_modbus_warning = Boolean(default=True)
    hide_control_disabled_warning = Boolean(default=True)
    hide_start_button = Boolean(default=True)
    hide_stop_button = Boolean(default=True)
    hide_reset_button = Boolean(default=True)
    hide_setpoint = Boolean(default=True)
    trip_label = String(default="Drive tripped")
