# Optidrive E3 / Techtop TTA-3 reference

Notes from the Invertek manuals and bench testing, 2026-09-29. The TTA-3 is a
rebadged Invertek Optidrive E3 (ODE-3). The manuals are Invertek's copyright
and are not stored in this repo; use the links below.

## Sources

- **Optidrive E3 Advanced Technical Manual, Issue 03**:
  <http://idt.com.tr/wp-content/pdfs/e3/Optidrive%20E3%20Advanced%20Technical%20Manual%20Issue%2003.pdf>
  - §3.2 Modbus RTU: supported function codes, parameters excluded from Modbus
  - §3.5.10 Communications Configuration (P-36): packed register layout
- **Optidrive ODE-3 User Guide, Version 2.00**:
  <https://fusionfluid.com/images/Documents/manuals-documentation/invertek_vfd/Invertek_Manual-v200.pdf>
  - Parameter table (P-36, P-37, P-38), §5.3 changing parameters on the keypad

## Modbus function codes (ATM §3.2)

The E3 documents only:

| Code | Name |
|---|---|
| FC03 | Read Holding Registers |
| FC06 | Write Single Holding Register |

Bench findings on this firmware:

| Write | Registers | Result |
|---|---|---|
| FC16, one value | Control word (1), setpoint (2) | Accepted (undocumented) |
| FC16, several values | Control block | Refused |
| FC16, one value | Parameters (128 + P-xx) | Refused: `IllegalFunction` (exception response 0x90) |
| FC06 | Parameters (128 + P-xx) | Accepted, stored, read back |
| FC06, out of range | e.g. P-08 = 2.3 A on a 2.2 A drive | Refused: `IllegalValue` (exception response 0x86) |

Parameter writes were accepted with the drive enabled (DI1 high, P-12 = 3)
and stopped. The manual notes that some parameters cannot be changed while the
drive is enabled.

## Parameters over Modbus (ATM §3.2)

All user-adjustable parameters are holding registers at **128 + P-number**
(P-08 is register 136, wire address 135), **except the three P-36 indices**:
drive address, baud rate and comms-loss timeout. Those can only be set on the
keypad.

## P-36 register layout (ATM §3.5.10)

Register 164 reads P-36 as one packed 16-bit word:

| Bits | Field |
|---|---|
| 0-7 | Drive address, 1-63 |
| 8-11 | Baud rate setting |
| 12-15 | Trip (comms-loss) configuration |

| Baud setting | Modbus RTU |
|---|---|
| 0, 1, 6-10 | 115.2 kbps |
| 2 | 9.6 kbps |
| 3 | 19.2 kbps |
| 4 | 38.4 kbps |
| 5 | 57.6 kbps |

| Trip setting | Keypad | Behaviour |
|---|---|---|
| 0 | 0 | Comms-loss watchdog disabled |
| 1-4 | t 30 / t 300 / t 1000 / t 3000 | Watchdog of 30 / 300 / 1000 / 3000 ms, **trip** on comms loss |
| 5-8 | r 30 / r 300 / r 1000 / r 3000 | Watchdog of 30 / 300 / 1000 / 3000 ms, **ramp to stop** on comms loss |

The watchdog runs only while the drive is enabled and is fed by writes to
register 1 (the control word). The app writes the control word every
*Poll Interval* while in Modbus control.

Bench drive: `0x4601` = address 1, 115.2 kbps, trip setting 4 (`t 3000`,
the factory default). Kept deliberately: a pump that lost comms stays tripped
until someone resets it, rather than restarting by itself.

## Keypad access to extended parameters

Parameters above P-14 are hidden until P-14 holds the access code in P-37
(default 101). P-38 = 1 locks all parameter edits. The bench drive reads
P-14 = 0 (locked), P-37 = 101, P-38 = 0.
