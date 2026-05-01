# Motor Control Subsystem

This document describes the motor control subsystem of the hybrid leg MicroPython
application. It covers the impedance controller, the Novanta Everest servo driver,
kinematic transmission calculations, control parameter mapping, and servo
activation/deactivation sequences.

---

## Impedance Controller (controller.py)

The `Controller` class implements a PD impedance controller that runs in the ISR
loop. It computes a current command for the Everest servo based on position error,
velocity, error derivative, and a torque bias term.

### Controller Equation

```
Output = (K * error - B * velocity - D * d(error)/dt + T_bias) / KT
```

Where:

| Symbol | Meaning |
|--------|---------|
| `K` | Stiffness gain (Nm/deg) |
| `error` | `SET_POINT - FEEDBACK_VALUE` (degrees) |
| `B` | Damping gain (Nm*s/deg) |
| `velocity` | Joint velocity from the velocity array (deg/s) |
| `D` | Derivative gain |
| `d(error)/dt` | `delta_error / delta_time` |
| `T_bias` | Constant torque bias (Nm) |
| `KT` | Motor torque constant (Nm/A) -- divides to convert torque to current |

### Parameter Array Layout

`Controller.controller_parameters` is a 20-element `array.array('f')`. Each
element is referenced by a named constant:

| Index | Constant | Description |
|------:|----------|-------------|
| 0 | `ERROR` | Current position error (computed each call) |
| 1 | `B` | Damping gain |
| 2 | `DELTA_ANGLE` | Change in joint angle (currently unused in output) |
| 3 | `D` | Derivative gain |
| 4 | `DELTA_ERROR` | Change in error between calls (computed each call) |
| 5 | `DELTA_TIME` | Time step in seconds (clamped to 5 ms max) |
| 6 | `KT` | Motor torque constant |
| 7 | `OLD_ERROR` | Previous error value for derivative calculation |
| 8 | `OLD_ANGLE` | Previous feedback angle |
| 9 | `FEEDBACK_VALUE` | Current joint angle from encoder |
| 10 | `SET_POINT` | Target joint angle (degrees) |
| 11 | `ONE_MILLION` | Constant `1000000.0` (time conversion helper) |
| 12 | `K` | Stiffness gain |
| 13-14 | *(reserved)* | Position estimate / predicted position error (unused) |
| 15 | `VELOCITY_ESTIMATE` | Velocity estimate (may be mapped via CAN) |
| 16-18 | *(reserved)* | Velocity integrator, KP/KI tracking loop (unused) |
| 19 | `T_BIAS` | Torque bias -- position hard-coded in CAN setup |

### Methods

**`controller_call(feedback_array, velocity_array)`**

Main computation method, called from the ISR. Steps:

1. Reads `feedback_array[0]` into `FEEDBACK_VALUE`.
2. Computes `DELTA_TIME` from `time.ticks_us()`, clamped to a 5 ms maximum to
   guard against scheduling jitter or negative tick differences.
3. Computes `error = SET_POINT - FEEDBACK_VALUE`.
4. Computes `delta_error = error - OLD_ERROR`, stores current error for next call.
5. Evaluates the controller equation and writes the result to `output_array[0]`.

**`clear_all()`**

Zeros `SET_POINT`, `K`, `B`, `D`, `OLD_ERROR`, `OLD_ANGLE`, and `T_BIAS`. Used
when the controller must be fully disabled.

**`clear_parameters_with_damping_minimum()`**

Same as `clear_all()` but sets `B = 0.1` instead of `0`. This provides a small
residual damping even when impedance control is nominally off, preventing
completely free-swinging behavior.

---

## Everest Servo Driver (novanta.py)

The `Everest` class manages SPI communication with the Novanta Everest motor
servo controller. It handles booting, register configuration, cyclic data
exchange, and CiA402 state machine transitions.

### Communication Modes

| Mode | Value | Description |
|------|------:|-------------|
| BOOT | 0 | Active during hardware reset. The microcontroller holds `boot_sync1` low, pulses the reset pin, then waits for the interrupt pin to signal boot completion. |
| CONFIG | 1 | Register read/write via call-response SPI frames (send a command frame, then read the reply on the next SPI exchange). Used for initial register configuration. |
| CYCLIC | 2 | Full-duplex SPI exchange every ISR tick. A PICO frame is sent while a POCI frame is simultaneously received. Config-channel fields are embedded in the same frame alongside cyclic data. |

Mode is stored in `Everest.communication_mode` and changed by writing to register
`0x640` (Communication Mode Status).

### CiA402 State Machine

The Everest follows the CiA402 (CANopen drives) state machine. States and their
status word bit patterns:

| State | Status Word Pattern | Identifier |
|-------|---------------------|------------|
| NOT READY TO SWITCH ON | `x0xx 0000` | `STATE_NOT_READY_TO_SWITCH_ON` (1) |
| SWITCH ON DISABLED | `x1xx 0000` | `STATE_SWITCH_ON_DISABLED` (2) |
| READY TO SWITCH ON | `x01x 0001` | `STATE_READY_TO_SWITCH_ON` (3) |
| SWITCHED ON | `x01x 0011` | `STATE_SWITCHED_ON` (4) |
| OPERATION ENABLED | `x01x 0111` | `STATE_OPERATION_ENABLED` (5) |
| QUICK STOP ACTIVE | `x00x 0111` | `STATE_QUICK_STOP_ACTIVE` (6) |
| FAULT REACTION ACTIVE | `x0xx 1111` | `STATE_FAULT_REACTION_ACTIVE` (7) |
| FAULT | `x0xx 1000` | `STATE_FAULT` (8) |

Key transitions used in this system:

| Transition | From | To | Control Word |
|:----------:|------|-----|:------------:|
| 2 | SWITCH ON DISABLED | READY TO SWITCH ON | CW = 6 |
| 3 | READY TO SWITCH ON | SWITCHED ON | CW = 7 |
| 4 | SWITCHED ON | OPERATION ENABLED | CW = 15 |
| 5 | OPERATION ENABLED | SWITCHED ON | CW = 7 |
| 7 | READY TO SWITCH ON | SWITCH ON DISABLED | CW = 0 |
| 8 | OPERATION ENABLED | READY TO SWITCH ON | CW = 6 |
| 9 | OPERATION ENABLED | SWITCH ON DISABLED | CW = 0 |
| 11 | OPERATION ENABLED | QUICK STOP ACTIVE | CW = 2 |
| 15 | FAULT | SWITCH ON DISABLED | CW = 128 |

### Key Methods by Mode

**Boot mode:**

- `hard_reset()` -- Pulses the reset pin with proper `boot_sync1` sequencing.
  Waits up to 5000 ms for the interrupt pin to signal readiness, then transitions
  to CONFIG mode. After reset, `boot_sync1` is reconfigured from output to input
  (it becomes the servo's sync pulse output).

**Config mode:**

- `set_register(register, value)` -- Writes a value to any register, auto-detecting
  the data type (float, uint, int) via `get_register_info()`.
- `enable_cyclic_mode()` -- Writes `2` to register `0x640`, switching from CONFIG
  to CYCLIC.
- `state_transition(n)` / `enter_state(state)` -- Perform CiA402 state transitions
  by writing to the control word register.

**Cyclic mode:**

- `send_receive_cyclic()` -- Full-duplex SPI exchange. Computes CRC on the PICO
  buffer, waits for `SPI_ready` (set by the ISR), then performs
  `spi.write_readinto(pico_buffer, poci_buffer)`. CRC verification on the
  received POCI frame is currently disabled for speed.
- `operation_enable_cyclic()` -- Performs transitions 2 -> 3 -> 4 in cyclic mode
  to reach OPERATION ENABLED. Each transition writes the appropriate control word
  into the PICO config data, exchanges frames, then polls the POCI status word
  until the target state is confirmed. Returns `0` on success, or the status word
  on failure.
- `operation_disable_cyclic()` -- Performs transition 8 (OPERATION ENABLED ->
  READY TO SWITCH ON). Returns `0` on success.

### Cyclic Register Mechanism (PICO/POCI Channels)

Cyclic data exchange uses the `Channel` class. Two channel instances are created
at initialization:

- **PICO** (Peripheral In, Controller Out) -- Data sent to the servo (e.g.,
  target current, control word).
- **POCI** (Peripheral Out, Controller In) -- Data received from the servo (e.g.,
  encoder position, status word, velocity).

#### Channel Buffer Layout

```
Bytes [0:2]                  Header (address, command, pending bit)
Bytes [2:10]                 Config data (4 words, used for register access)
Bytes [10:10+cyclic_size]    Cyclic data (0-32 words of mapped registers)
Bytes [last 2]               CRC-16 checksum
```

Memory views (`header_mv`, `config_data_mv`, `cyclic_data_mv`, `CRC_data_mv`,
`CRC_sum_mv`) provide zero-copy access into the underlying `bytearray`.

#### Dynamic Property Generation

When a PICO or POCI channel is created, `_generate_property_methods()` iterates
over the cyclic register dictionary and creates Python `property` descriptors on
the Channel class for each register. For example, if `status_word` is a POCI
register, it becomes accessible as `channel_poci.status_word`.

Each property getter reads bytes from `cyclic_data_mv` at the register's byte
offset, performs Novanta's word-swapped byte ordering, and returns a native Python
value. Each setter does the reverse. Data types supported: `INT16`, `UINT16`,
`INT32`, `UINT32`, and `FLOAT`.

The method also writes the cyclic register configuration to the servo via
registers `0x651+` (PICO) or `0x661+` (POCI), and sets the total register count
at `0x650` or `0x660`.

### CRC Verification

Two implementations are available, selected at initialization via
`use_hardware_crc`:

| Method | Function | Speed | Notes |
|--------|----------|-------|-------|
| Hardware | `crc_arm()` | ~7 us | Uses the STM32 CRC peripheral (CRC-16-CCITT, polynomial `0x1021`). Initializes via `init_crc_arm()` which enables the AHB clock and configures the 16-bit polynomial. Uses `@micropython.viper` for direct memory access. |
| Software | `crc_software()` | ~160 us | Pure-Python CRC-16-CCITT (XModem). ISR-safe but significantly slower. |

Both operate on `data[0:size]` and return a 16-bit CRC value. A return of `0`
when run over a buffer including its own CRC indicates a valid frame.

---

## Transmission Ratio Calculations

These methods in `HybridLeg` compute the mechanical transmission ratio `n`
between the motor and the joint, as a function of joint angle.

### Hybrid Knee -- Crank-Slider Mechanism

**Method:** `calculate_hybrid_knee_transmission(joint_angle)`

Uses a crank-slider kinematic model with parameters from `CONST_DICT`:

| Parameter | Key |
|-----------|-----|
| Crank helper | `CRANK_HELPER_ARRAY[0]` |
| Pivot bar length | `PIVOT_BAR_LENGTH` |
| Angle offset | `CVT_ANGLE_ALPHA` |
| Crank arm length | `CVT_ARM_LENGTH` |
| Gear ratio | `KNEE_GEAR_RATIO` |
| Screw lead | `ROLLVIS_SCREW_LEAD` |

Calculation:

```
x = (joint_angle + CVT_ANGLE_ALPHA) * pi / 180

intermediate = CVT_ARM_LENGTH * sin(x) - CRANK_HELPER_ARRAY[0]

numerator = CVT_ARM_LENGTH * (
    sin(x) - cos(x) * intermediate
              / sqrt(PIVOT_BAR_LENGTH^2 - intermediate^2)
)

transmission = numerator * 2 * pi * KNEE_GEAR_RATIO / ROLLVIS_SCREW_LEAD
```

### Fox Knee -- Linkage Mechanism

**Method:** `calculate_fox_knee_transmission(joint_angle)`

Uses pre-computed terms from `CONST_DICT['FOX_TRANSMISSION_CALCULATION_TERMS']`,
an array where:

- `terms[0]` -- angular offset
- `terms[1]`, `terms[2]`, `terms[3]` -- linkage geometry constants

```
gamma_prime = radians(joint_angle - terms[0])

transmission = terms[1] * sin(gamma_prime)
               / sqrt(terms[2] + terms[3] * cos(gamma_prime))
```

### Polycentric Ankle -- Polynomial

The ankle uses a polynomial mapping between joint angle and encoder counts,
evaluated via `assembler_functions.calculate_polynomial()` with coefficients from
`CONST_DICT['P2_JOINT_ENCODER_POLYNOMIAL_COEFFICIENTS_INVERTED']`. This is used
within `update_control_parameters()` to convert the set point angle to encoder
counts.

---

## Control Parameter Update

**Method:** `update_control_parameters()` in `hybrid_leg.py`

This method maps the impedance parameters from CAPS (K, B, and set point from
`controller_parameters`) into Everest-compatible servo gains (KPP, KPV, KPT) and
writes them to `PARAM_DICT`.

### Input Values

| Source | Index | Meaning |
|--------|------:|---------|
| `controller_parameters[12]` | K | Stiffness from CAPS |
| `controller_parameters[1]` | B | Damping from CAPS (clamped to max 0.6) |
| `controller_parameters[10]` | SET_POINT | Target angle from CAPS |
| `controller_parameters[19]` | T_BIAS | Torque bias from CAPS |

### Intermediate Values

| Symbol | Source |
|--------|--------|
| `k_t` | `CONST_DICT['TORQUE_CONSTANT_LARGE_MOTOR']` |
| `n` | `PARAM_DICT['LARGE_MOTOR_TRANSMISSION_RATIO'][0]` (computed by transmission functions) |
| `c` | `KNEE_JOINT_ENC_COEF` (knee devices) or `ANKLE_JOINT_ENC_COEF` (ankle devices) -- encoder degrees per count |

### KPP, KPV, KPT Calculations

**KPV (velocity loop proportional gain):**

| Device | Formula |
|--------|---------|
| Hybrid Knee | `KPV = 360.0 * B / n` |
| Fox Knee | `KPV = 360.0 * B` (no division by `n`) |
| Polycentric Ankle | `KPV = 360.0 * B / n` |

**KPP (position loop proportional gain):**

```
if B == 0:
    KPP = 0
else:
    KPP = c * K / KPV
```

**KPT (torque loop proportional gain):**

```
KPT = 1 / (n * k_t)
```

**Set point:**

- Knee devices: `EVEREST_SET_POINT = int(SET_POINT / c)`
- Polycentric Ankle: Set point angle is converted to encoder counts via the
  inverted polynomial.

**Torque bias:**

```
EVEREST_TORQUE_BIAS = controller_parameters[19]   (T_BIAS)
```

---

## Everest Activation / Deactivation

### activate_everest()

Brings the Everest from its idle state to OPERATION ENABLED so the motor can be
driven.

**Sequence:**

1. For knee devices, disable the braking circuit FETs
   (`braking_circuit_fet_enable(0)`) -- this must happen before activation.
2. Assert the STO (Safe Torque Off) enable line high via `everest_enable(1)`.
3. Wait 5 ms (via `alarm_clock(5000)`) to allow the STO lines to stabilize.
4. Call `servo.operation_enable_cyclic()`, which performs CiA402 transitions:
   - `SWITCH ON DISABLED` -> `READY TO SWITCH ON` (transition 2)
   - `READY TO SWITCH ON` -> `SWITCHED ON` (transition 3)
   - `SWITCHED ON` -> `OPERATION ENABLED` (transition 4)
5. On success (return value `0`):
   - `EVEREST_FAILURE_FLAG = False`
   - `EVEREST_OPERATION_IS_ENABLED = True`
6. On failure (non-zero status word returned):
   - `EVEREST_FAILURE_FLAG = True`
7. Always clears `ACTIVATING_EVEREST_FLAG = False` at the end.

### deactivate_everest()

Brings the Everest back to an idle state from OPERATION ENABLED.

**Sequence:**

1. Call `servo.operation_disable_cyclic()`, which performs CiA402 transition:
   - `OPERATION ENABLED` -> `READY TO SWITCH ON` (transition 8)
2. On success (return value `0`):
   - Immediately lower STO line via `everest_enable(0)`.
   - For knee devices, re-enable braking circuit FETs
     (`braking_circuit_fet_enable(1)`) to keep STO lines consistent.
   - `EVEREST_FAILURE_FLAG = False`
   - `EVEREST_OPERATION_IS_ENABLED = False`
3. On failure:
   - `EVEREST_FAILURE_FLAG = True`
4. Always clears `DEACTIVATING_EVEREST_FLAG = False` at the end.

### Flag Coordination

| Flag | Purpose |
|------|---------|
| `ACTIVATING_EVEREST_FLAG` | Set `True` by `large_motor_active_mode()` to request activation; cleared by `activate_everest()` when done. |
| `DEACTIVATING_EVEREST_FLAG` | Set `True` by `idle_brake_mode()` to request deactivation; cleared by `deactivate_everest()` when done. |
| `EVEREST_OPERATION_IS_ENABLED` | `True` when the Everest is in OPERATION ENABLED and ready to accept torque commands. |
| `EVEREST_FAILURE_FLAG` | `True` if the last activation or deactivation attempt failed. |
| `LARGE_MOTOR_DRIVE_FLAG` | `True` when the ISR should actively drive the main motor with computed current commands. |

---

## Motor Modes

### large_motor_active_mode()

Called by the state machine when the motor should be actively driving the joint.

1. **Disable braking** (knee devices only): Sets `APPLIED_BRAKE_PERCENTAGE = 0`
   and drives the braking circuit PWM to 0%.
2. **Check Everest state:**
   - If `EVEREST_OPERATION_IS_ENABLED` is `True`, sets
     `LARGE_MOTOR_DRIVE_FLAG = True` to begin driving.
   - Otherwise, if `ACTIVATING_EVEREST_FLAG` is not already set, sets it to
     `True` to request activation on the next cycle.

### idle_brake_mode()

Called by the state machine when the motor should not be actively driven. The
joint may free-swing or have passive damping applied through the braking circuit.

1. **Disable motor driving:** Sets `LARGE_MOTOR_DRIVE_FLAG = False`.
2. **Request deactivation:** If the Everest is enabled and not already
   deactivating, sets `DEACTIVATING_EVEREST_FLAG = True`.
3. **Apply braking** (knee devices only, when Everest is disabled):
   - Reads the dynamic brake percentage (computed from velocity) and the CAPS
     brake command.
   - Applies the larger of the two values, capped at 100%.
   - If `DYNAMIC_BRAKE_STATE` is not active, sets braking to 0% (completely
     free-swinging).
   - Writes the resulting percentage to the braking circuit PWM channel.

---

## See Also

- [ISR Reference](./isr-reference.md)
- [CAN and CAPS Protocol](./caps-and-can.md)
- [Data Dictionaries](../../documentation/data-dictionaries.md)
- [Device Configuration](../../documentation/device-configuration.md)
