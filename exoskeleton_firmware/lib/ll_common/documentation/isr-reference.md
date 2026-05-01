# ISR Reference

All real-time behavior in this system is driven by five timer ISRs running concurrently on a single core. This document details each ISR's execution flow, timing, and device-specific branching.

All timer callbacks are attached in `main.py` lines 564–581. Interrupt priorities are managed via `nvic.py` (all set to priority level 5).

**ISR safety rules apply to all code documented here.** See the root `CLAUDE.md` for the full list of constraints (no allocation, no blocking I/O, no exceptions except outer try/except wrappers).

---

## Timer ISR Summary

| ISR | Location | Frequency | Purpose |
|-----|----------|-----------|---------|
| `isr_joint_controller` | `hybrid_leg.py:614` | 500 Hz (knee) / 250 Hz (ankle) | Servo SPI exchange, joint calculations, impedance control |
| `timer_loop` | `main.py:271` | 250 Hz (knee) / 500 Hz (ankle H747) | Main app logic, watchdog, CAPS stream control |
| `tick_caps` | `hybrid_leg.py:932` | 250 Hz | Send telemetry CAN messages to CAPS |
| `send_message` | `can_sender.py` | 1000 Hz | Dequeue and transmit buffered CAN messages |
| `tick_imu` | `hybrid_leg.py:886` | 50 Hz | Read IMU sensors, send IMU CAN messages |

Additionally, if PRIMARY_CONTROLLER and state machine is loaded:

| ISR | Location | Frequency | Purpose |
|-----|----------|-----------|---------|
| `execute_state_machine` | `state_machine.py:134` | 100 Hz | Run loaded .psm finite state machine |

---

## ISR 1: isr_joint_controller (500 Hz / 250 Hz)

**Source:** `hybrid_leg.py:614`
**Decorator:** `@micropython.native`
**Timer:** `TIMER_MOTOR_CONTROL`

This is the most timing-critical ISR. It handles full-duplex SPI communication with the Everest servo and runs the impedance controller.

### Execution Flow (per tick)

```
1. timing_pin(1)
2. Cache PARAM_DICT and FLAG_DICT as locals
3. IF not activating/deactivating Everest:
   a. servo.send_receive_cyclic()              ← SPI exchange #1
   b. Read POCI registers:
      - actual_position → MAIN_JOINT_ENCODER_VALUE (with rollunder handling)
      - actual_velocity → motor velocity (×360 for deg/s)
      - current_quadrature_value → LARGE_MOTOR_REFERENCE_CURRENT
      - status_word → everest_status_word
      - bus_voltage_value → BATTERY_VOLTS
      - motor_temperature_value → MOTOR_TEMP (Fox Knee only)
   b2. FTS polling (Fox Knee only, if fts_sensor present):
      - fts_sensor.get_force_moment() → FTS_FZ, FTS_MY
   c. JOINT ANGLE calculation (device-specific):
      - Knee: ENCODER × KNEE_JOINT_ENC_COEF (linear)
      - Ankle: calculate_polynomial(P2_JOINT_ENCODER_POLYNOMIAL_COEFFICIENTS)
   d. TRANSMISSION RATIO calculation (device-specific):
      - Hybrid Knee: calculate_hybrid_knee_transmission() (crank-slider)
      - Fox Knee: calculate_fox_knee_transmission() (linkage)
      - Ankle: calculate_polynomial(P2_TRANSMISSION_POLYNOMIAL_COEFFICIENTS)
   e. JOINT SPEED: compute_joint_velocity() (except Fox Knee which uses direct assignment)
   f. UPDATE CONTROL PARAMETERS (Knee devices only):
      - update_control_parameters() → sets KPP, KPV, KPT, SET_POINT, TORQUE_BIAS
   g. Write PICO registers (Knee devices):
      - position_loop_kp, velocity_loop_kp, torque_loop_kp
      - position_set_point, torque_loop_input_offset
   h. servo.send_receive_cyclic()              ← SPI exchange #2
4. ELIF activating → activate_everest()
5. ELIF deactivating → deactivate_everest()
6. IMPEDANCE CONTROL (all devices):
   - everest_pb.controller_call(JOINT_ANGLE, JOINT_SPEED_ARRAY)
   - Output: LARGE_MOTOR_CURRENT_COMMAND = output / transmission_ratio × rotation_direction
7. ANKLE MOTOR DRIVE (ankle devices only):
   - If LARGE_MOTOR_DRIVE_FLAG: write current_quadrature_set_point
   - Adjust JOINT_ANGLE by ANKLE_NEUTRAL_OFFSET
8. DYNAMIC BRAKING (knee devices only):
   - Sigmoid function: 200 - 200/(1 + exp(-angle/rate))
   - Updates DYNAMIC_BRAKE_PERCENTAGE
9. CONTROLLER_TIMER = elapsed time
10. timing_pin(0)
```

### Device-Specific Branching

| Step | Hybrid Knee | Fox Knee | Polycentric Ankle |
|------|------------|----------|-------------------|
| Angle calc | Linear (enc × coef) | Linear (enc × coef) | Polynomial (3rd order) |
| Velocity | via motor_velocity object | Direct to JOINT_SPEED_ARRAY | via motor_velocity object |
| Transmission | Crank-slider kinematic | Linkage kinematic | Polynomial (6th order) |
| Joint speed | compute_joint_velocity() | Already computed | compute_joint_velocity() |
| Control params | update + write PICO | update + write PICO | Commented out |
| Motor drive | Via position mode | Via position mode | Direct current set point |
| Dynamic braking | Yes (sigmoid) | Yes (sigmoid) | No |
| Motor temp | No | Yes | No |
| FTS polling | No | Yes (if fts_sensor present) | No |

---

## ISR 2: timer_loop (250 Hz / 500 Hz)

**Source:** `main.py:271`
**Timer:** `TIMER_MAIN_LOOP`

See [Main Loop Reference](../../documentation/main-loop.md) for full documentation. Brief summary:

1. LED/buzzer status update
2. Watchdog check (75 ms timeout)
3. Everest error check
4. Battery voltage monitoring (knee only, low < 22V, critical < 20.5V)
5. CAPS start/stop stream handling
6. Streaming mode: CAPS command dispatch via `micropython.schedule()`, motor state logic
7. CAN ACK
8. Brake percentage update
9. Loop timing

---

## ISR 3: tick_caps (250 Hz)

**Source:** `hybrid_leg.py:932`
**Timer:** `TIMER_CAPS_SEND`

### Execution Flow (per tick)

```
1. timing_pin(1)
2. CAN bus health check:
   - can.info() → CAN_INFO_HOLDER
   - If bus off counter increased → set CAN_RESTART_FLAG
   - If bus off count ≥ 100 → activate 'can_bus_repeatedly_failed'
3. KNEE DEVICES (alternating even/odd ticks via WIFI_COUNTER % 2):
   Even ticks:
     - can_out_mes1.send()   (angle, velocity, current → CAPS9_DATA 0x01F)
     - can_out_mes3.send()   (brake percentage → CAPS15_DATA 0x02B)
     - If FTS streaming: can_out_fts_data1.send() + can_out_fts_data2.send()
   Odd ticks:
     - Update ACTIVE_STATE from state machine
     - CAPS communication timing check (ping-pong protocol)
     - error_code_to_CAPS() → send error data
     - can_out_mes2.send()   (state, comm value → CAPS14_DATA 0x021)
4. ANKLE DEVICES (alternating even/odd ticks):
   Even ticks:
     - can_out_mes5.send()   (ankle data → CAPS13_DATA 0x0CF)
   Odd ticks:
     - error_code_to_CAPS() → send error data
5. WIFI_COUNTER = (WIFI_COUNTER + 1) % 1000
6. CAPS_COM_TIMER = elapsed time
7. timing_pin(0)
```

### CAPS Communication Timing Check (Knee only)

Ping-pong protocol on odd ticks:
1. If `CAPS_COMMUNICATION_VALUE == 0` and no reply pending → set to 1000, record CAPST1
2. When CAPS replies (via CAPS12 CAN message) → CAPS_REPLY_FLAG = True, record CAPST2
3. Compute round-trip time = CAPST2 - CAPST1
4. If > 350 ms → set CAPS_SLOW_PING_FLAG
5. Toggle value between 1000 and 2000 for next cycle
6. Limited to `CAPS_COMM_CHECK_LIMIT` (20) cycles

---

## ISR 4: send_message (1000 Hz)

**Source:** `can_sender.py`
**Timer:** `TIMER_CAN_EXTERNAL`

### Execution Flow

```
H747 (FDCAN):
  - While send buffer has entries AND FDCAN TX FIFO not full:
    - disable_irq()
    - Send from circular buffer
    - Advance sent_idx
    - enable_irq()

F767 (CAN):
  - While send buffer has entries AND mailbox available (up to 3):
    - disable_irq()
    - Send from circular buffer
    - Advance sent_idx
    - enable_irq()
```

IRQ is disabled/re-enabled around each send to prevent race conditions with other ISRs that call `cpy_mess_into_can_send()`.

---

## ISR 5: tick_imu (50 Hz)

**Source:** `hybrid_leg.py:886`
**Timer:** `TIMER_IMU` (primary controller only)

### Execution Flow

```
1. timing_pin(1)
2. assembler_functions.imu_isr(imu_isr_addresses)  ← assembly-optimized SPI exchange
3. Calculate barometric pressure from raw_pressure_array
4. Calculate altitude: 0.0044×p² - 17.175×p + 12877.0
5. If CAN_RESTART_FLAG → set seq_num_restart on IMU messages
6. can_out_mes_imu1.send()
7. can_out_mes_imu2.send()
8. Update MAX_IMU_TIMER if new max
9. timing_pin(0)
```

---

## ISR 6: execute_state_machine (100 Hz)

**Source:** `state_machine.py:134`
**Timer:** `TIMER_STATE_MACHINE` (primary controller only, if .psm file loaded)

### Execution Flow

```
1. update_sm_input_vars()    ← assembly: update timing, slopes, min/max from sensor data
2. check_sub_events()        ← assembly: evaluate event transition conditions
3. check_states()            ← assembly: evaluate state outputs, DOF calculations
4. If state changed → state_change_cleanup() ← assembly: reset variables
```

All operations use pre-allocated arrays accessed via memory addresses. The state machine reads from `sm_channels_current` (24-element float array populated by other ISRs).

---

## ISR Interaction Map

```
CAN RX callbacks (FIFO0/FIFO1) ──┐
  Writes: FLAG_DICT, PARAM_DICT,  │
  controller_parameters            │
                                   │
isr_joint_controller (500 Hz) ─────┤── Reads: FLAG_DICT, PARAM_DICT
  Writes: JOINT_ANGLE,             │   Writes: via SPI to servo
  JOINT_SPEED, TRANSMISSION_RATIO, │
  CURRENT_COMMAND, BATTERY_VOLTS   │
                                   │
timer_loop (250 Hz) ───────────────┤── Reads: FLAG_DICT, PARAM_DICT
  Writes: FLAG_DICT (stream,       │   Uses: micropython.schedule()
  motor state, watchdog)           │
                                   │
tick_caps (250 Hz) ────────────────┤── Reads: PARAM_DICT, FLAG_DICT
  Writes: CAN output messages      │   Reads: sm.active_state
                                   │
tick_imu (50 Hz) ──────────────────┤── Reads: IMU hardware
  Writes: BAR_BYTE_VALS, ALTITUDE  │   Writes: CAN output messages
                                   │
execute_state_machine (100 Hz) ────┘── Reads: sm_channels_current
  Writes: active_state, dof_outputs
```

---

## See Also

- [Main Loop Reference](../../documentation/main-loop.md) — Full timer_loop documentation
- [Motor Control](./motor-control.md) — Everest servo driver and impedance controller details
- [CAN and CAPS Protocol](./caps-and-can.md) — CAN message routing and CAPS NID definitions
- [Data Dictionaries](../../documentation/data-dictionaries.md) — FLAG_DICT and PARAM_DICT key reference
