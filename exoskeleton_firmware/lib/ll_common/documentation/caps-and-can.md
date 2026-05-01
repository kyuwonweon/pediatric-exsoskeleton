# CAN Bus and CAPS Protocol Subsystem

This document describes the CAN bus communication layer and CAPS (Clinical
Application Programming System) protocol used by the MicroPython prosthetic leg
controller. The subsystem handles real-time streaming of impedance parameters,
sensor data, and command messages between the CAPS PC application and the
embedded microcontroller.

## Source Files

| File | Purpose |
|------|---------|
| `caps_manager.py` | Defines all PDCP NID addresses and initializes CAPS handler objects |
| `can_class.py` | CAN bus manager: hardware filters, FIFO callbacks, message dispatch |
| `CAN_PDCP_Streaming_Message.py` | PDCP NID handler class for encoding/decoding streaming channels |
| `can_sender.py` | Circular-buffer CAN transmit driver with IRQ-safe send routines |
| `caps_command_handler.py` | Processes discrete commands received from CAPS (rehome, zero IMU, etc.) |

---

## CAN Bus Architecture

A single CAN bus is shared between the microcontroller and the CAPS PC. The
hardware peripheral used depends on the processor:

| Processor | Peripheral | CAN Type |
|-----------|-----------|----------|
| STM32H747 | FDCAN1 | FD-CAN (used in classical CAN mode) |
| STM32F767 | CAN1 | Classical CAN |

### Dual-FIFO Design

The CAN peripheral uses two receive FIFOs to separate traffic by priority:

- **FIFO 0** -- Impedance parameters and control messages (CAPS1, CAPS2, CAPS5
  through CAPS12). All device types receive on FIFO 0.
- **FIFO 1** -- Loadcell data (`LOAD_CELL1_DATA1`, `LOAD_CELL1_DATA2`).
  Enabled only on the **primary controller**.

Each FIFO has its own receive callback. The H747 variants are
`can_call_fifo0_H7` / `can_call_fifo1_H7`; the F767 variants are
`can_call_fifo0_F7` / `can_call_fifo1_F7`. All four follow the same pattern:
drain every pending message from the FIFO, look up the 8-bit NID in
`can_function_dict`, and dispatch to the appropriate processor method. On the
H747, the callbacks also explicitly re-enable the FDCAN new-message interrupt
line after draining (`FDCAN_IE` bit 0 for FIFO 0, bit 4 for FIFO 1).

### Hardware Filters

Filters are configured per device type and processor in the `canClass.__init__`
method.

**Hybrid Knee / Fox Knee (F767 -- MASK16 mode):**

| Filter | FIFO | NIDs Accepted |
|--------|------|---------------|
| 0 | 0 | CAPS1, CAPS2 |
| 1 | 0 | CAPS9, CAPS12 |
| 2 | 0 | CAPS7, CAPS8 |
| 3 | 0 | CAPS10, CAPS11 |
| 4 | 1 | LOAD_CELL1_DATA1, LOAD_CELL1_DATA2 |

**Hybrid Knee / Fox Knee (H747 -- MASK mode):**

| Filter | FIFO | NID Accepted |
|--------|------|--------------|
| 0 | 0 | CAPS1 |
| 1 | 0 | CAPS2 |
| 2 | 0 | CAPS7 |
| 3 | 0 | CAPS8 |
| 4 | 0 | CAPS9 |
| 5 | 0 | CAPS10 |
| 6 | 0 | CAPS11 |
| 7 | 0 | CAPS12 |
| 8 | 1 | LOAD_CELL1_DATA1 |
| 9 | 1 | LOAD_CELL1_DATA2 |
| 10 | 0 | FTS control NID (Fox Knee only, if FTS installed) |

**Polycentric Ankle (F767 -- MASK16 mode):**

| Filter | FIFO | NIDs Accepted |
|--------|------|---------------|
| 0 | 0 | CAPS5, CAPS6 |
| 1 | 0 | CAPS13, CAPS13 |

**Polycentric Ankle (H747 -- MASK mode):**

| Filter | FIFO | NID Accepted |
|--------|------|--------------|
| 0 | 0 | CAPS5 |
| 1 | 0 | CAPS6 |
| 2 | 0 | CAPS13 |

---

## Complete PDCP NID Map

Every CAN message on the bus is identified by an 11-bit Standard ID. The low
eight bits carry the NID value defined in `caps_manager.py`; the upper bits
encode priority and the mode bit (bit 8, `0x100`) used for acknowledgements.

### CAPS Output Devices (COD) -- Data FROM CAPS to Controller

These NIDs carry parameters that CAPS sends to the microcontroller.

| Name | NID (hex) | Purpose |
|------|-----------|---------|
| CAPS1_ID | `0x018` | Knee actuator control NID (start/stop, settings) |
| CAPS2_ID | `0x019` | Knee actuator data NID (K, B, E, Flex/Extend) |
| CAPS5_ID | `0x0C8` | Ankle actuator control NID |
| CAPS6_ID | `0x0C9` | Ankle actuator data NID (K, B, E, Flex/Extend) |
| CAPS7_ID | `0x0D4` | Knee dynamic brake control NID |
| CAPS8_ID | `0x0D5` | Knee dynamic brake data NID (Brake Set Point, Brake Angle, Brake Velocity) |
| CAPS10_ID | `0x0C0` | Buzzer data (frequency, volume, duration) |
| CAPS11_ID | `0x0F9` | CAPS communication check (outgoing ping) |
| CAPS12_ID | `0x0FA` | CAPS communication check (response) |

### CAPS Input Devices (CID) -- Data FROM Controller to CAPS

These NIDs carry sensor readings and state that the microcontroller sends to
CAPS. Each CID has a control NID and one or more data NIDs.

| Name | NID (hex) | Purpose |
|------|-----------|---------|
| CAPS9_ID | `0x01E` | Knee output primary -- control NID |
| CAPS9_DATA_ID | `0x01F` | Knee output primary -- data (`can_out_mes1`: joint angle, speed, currents) |
| CAPS14_ID | `0x020` | Knee output secondary -- control NID |
| CAPS14_DATA_ID | `0x021` | Knee output secondary -- data (`can_out_mes2`: battery volts, active state, button, comms) |
| CAPS15_ID | `0x02A` | CVT output primary -- control NID |
| CAPS15_DATA_ID | `0x02B` | CVT output primary -- data (`can_out_mes3`: brake percentage / motor temp) |
| CAPS13_ID | `0x0CE` | Ankle output primary -- control NID |
| CAPS13_DATA_ID | `0x0CF` | Ankle output primary -- data (`can_out_mes5`: joint angle, speed, currents) |
| CAPS16_ID | `0x0D0` | Ankle output secondary -- control NID (currently unused) |
| CAPS16_DATA_ID | `0x0D1` | Ankle output secondary -- data (`can_out_mes6`, currently unused) |
| IMU_ID | `0x02C` | IMU -- control NID |
| IMU_DATA1_ID | `0x02D` | IMU -- data 1 (`can_out_mes_imu1`: channels 16-19) |
| IMU_DATA2_ID | `0x02E` | IMU -- data 2 (`can_out_mes_imu2`: channels 20-23) |
| LOAD_CELL1_ID | `0x040` | Loadcell -- control NID |
| LOAD_CELL1_DATA1_ID | `0x041` | Loadcell -- data 1 (channels 1-3) |
| LOAD_CELL1_DATA2_ID | `0x042` | Loadcell -- data 2 (channels 4-6) |
| FTS control NID | `0x044` | FTS -- control NID (start/stop streaming; per-unit, from `fts_parameters.py`) |
| FTS data NID 1 | `0x045` | FTS -- data 1 (`can_out_fts_data1`: [zero, zero, Fz], 16-bit, Fox Knee only) |
| FTS data NID 2 | `0x046` | FTS -- data 2 (`can_out_fts_data2`: [zero, My, zero], 16-bit, Fox Knee only) |
| FTS_AND_ERROR_ID | `0x0E8` | Error flag NID (reserved) |

---

## PDCP Message Format (CAN_PDCP_Streaming_Message.py)

### CanPdcpNidHandler Class

`CanPdcpNidHandler` is the central abstraction for a single PDCP channel. Each
instance represents one NID and is responsible for encoding outgoing data or
decoding incoming data. There are two flavours:

- **Control NID** (`is_control_nid=True`) -- Read-only. Stores the raw 8-byte
  payload for inspection by message processors. Tracks `payload_length`,
  `received_id`, and `can_ack_flag` for the acknowledgement mechanism.
  The `send` attribute is set to `None`.
- **Streaming/Data NID** (`is_control_nid=False`) -- Carries multi-channel
  scaled data. `send` is bound to `_send_message()`.

### Constructor Parameters

```
CanPdcpNidHandler(can_nid, adc_resolution, zero_and_gain_array, outmap, can_sender, is_control_nid=False)
```

| Parameter | Description |
|-----------|-------------|
| `can_nid` | Combined priority + NID, e.g. `0x300 + CAPS2_ID`. The upper nibble encodes priority: `0x100` for control, `0x300` for impedance streaming, `0x400` for sensor streaming. |
| `adc_resolution` | `12` or `16`. Determines channel count and packing format. |
| `zero_and_gain_array` | Flat `array('f', ...)` of interleaved `[zero, gain, zero, gain, ...]` pairs. Zero is scaled to the midpoint of the ADC range; gain is the full-scale divisor for the physical unit. |
| `outmap` | `array('i', ...)` of memory addresses where decoded values are written (input) or read from (output). |
| `can_sender` | Reference to the `canSender` instance used by `_send_message()`. |

### 12-bit vs 16-bit Encoding

| Mode | Channels per Frame | Assembler Read Function | Assembler Write Function |
|------|--------------------|------------------------|-------------------------|
| 12-bit | 4 | `fast_caps_read_12bit_4chans` | `fw_CAPS_u12bit` |
| 16-bit | 3 | `fast_caps_read_16bit` | `fw_CAPS_u16bit` |

In 12-bit mode, four 12-bit samples are packed into 6 bytes (bytes 1-6 of the
8-byte CAN payload), leaving byte 0 for the sequence number and byte 7 unused
by channel data. In 16-bit mode, three 16-bit samples occupy bytes 1-6.

### Scaling: zero_and_gain_array

For each channel _i_ the `zero_and_gain_array` contains two values:

- `zero_and_gain_array[i * 2]` -- Zero offset, computed as
  `int(zero_scale_factor * (2^adc_resolution - 1))`. Typically `0.5` is
  supplied, placing the zero at the midpoint of the ADC range.
- `zero_and_gain_array[i * 2 + 1]` -- Gain value. Stored into a separate
  `gain_map` array for use by the assembler write function.

Gain values from `caps_manager.py` encode the physical range of each channel.
For example, CAPS2 (knee impedance) uses gains of `[41.12, 411.2, 11.35,
8.224]` representing stiffness (Nm/deg), damping (Nm*s/deg), equilibrium
(degrees), and flex/extend mode respectively.

### Sequence Number Management

Byte 0 of every streaming frame is a sequence number.

- On `_send_message()`, the sequence number is incremented by 1 after the
  channel data is packed. It wraps naturally at 255 (unsigned byte).
- If `seq_num_restart_flag` is `True`, byte 0 is forced to 0 before
  incrementing, producing a value of 1. This signals to the receiver that a
  stream restart has occurred.
- The `zero()` method clears all 8 bytes of the data buffer to 0, which also
  resets the sequence number.
- The `zero_and_clear()` method runs the full decode pipeline with an
  all-zeros buffer, which both resets the data payload and writes zero-scaled
  values to the output map addresses.

---

## CAN Bus Manager (can_class.py)

`canClass` is instantiated once during system initialization. It owns:

1. Hardware filter setup (described above).
2. FIFO receive callbacks (one per FIFO per processor).
3. A dispatch dictionary (`can_function_dict`) mapping 8-bit NID values to
   processor methods.
4. The ACK mechanism for acknowledged CAPS messages.

### Callback Dispatch via can_function_dict

```python
self.can_function_dict = {
    CAPS1_ID:  process_CAPS_Setting,    # 0x018
    CAPS5_ID:  process_CAPS_Setting,    # 0x0C8
    CAPS7_ID:  process_CAPS_Setting,    # 0x0D4
    CAPS11_ID: process_CAPS_Setting,    # 0x0F9
    CAPS10_ID: process_CAPS_Buzzer,     # 0x0C0
    CAPS2_ID:  process_CAPS_Streaming,  # 0x019
    CAPS6_ID:  process_CAPS_Streaming,  # 0x0C9
    CAPS8_ID:  process_CAPS_Streaming,  # 0x0D5
    CAPS9_ID:  process_CAPS_Message,    # 0x01E
    CAPS12_ID: process_CAPS_Message,    # 0x0FA
    CAPS13_ID: process_CAPS_Message,    # 0x0CE
}
# Primary controller only:
#   LOAD_CELL1_DATA1_ID -> process_LOAD_CELL_DATA
#   LOAD_CELL1_DATA2_ID -> process_LOAD_CELL_DATA
# Fox Knee with FTS installed:
#   fts_nid_control -> process_FTS_Setting
```

### Message Processors

#### process_CAPS_Setting (CAPS1, CAPS5, CAPS7, CAPS11)

Handles start/stop streaming commands and motor disable messages:

- `0x50` with payload length 2: Sets `CAPS_START_STREAM_FLAG`.
- `0x51` with payload length 3: Sets `CAPS_STOP_STREAM_FLAG`.
- `0x88` + `0x01`: Motor disable -- clears `LARGE_MOTOR_STATE`.
- `0x80` (primary controller only): Fob value calculation via
  `assembler_functions.fob_value_calc`.

#### process_CAPS_Streaming (CAPS2, CAPS6, CAPS8)

Decodes impedance parameter frames via `CanPdcpNidHandler.process_can_message`,
which unpacks the ADC-encoded channels and writes scaled float values directly
into memory (e.g., `everest_pb.controller_parameters`). After decoding,
`execute_flag_logic` runs:

- Resets the watchdog timer (`watchdog_timer0`).
- Records ISR timing into `PARAM_DICT`.
- For CAPS2 (knee): enables `LARGE_MOTOR_STATE` when stiffness exceeds 0.001;
  zeroes stiffness values below 0.1.
- For CAPS6 (ankle, polycentric devices): applies a minimum damping floor of
  0.1, computes the ankle equilibrium offset with end-stop limiting via
  `eq_offset_and_limiter`, and manages `LARGE_MOTOR_STATE`.

#### process_CAPS_Message (CAPS9, CAPS12, CAPS13)

Handles discrete command messages and PDCP commands:

- **CAPS12**: Records `CAPST2` timestamp and sets `CAPS_REPLY_FLAG` for the
  ping/latency check, then returns.
- **Single-byte payloads** (CAPS9 or CAPS13):
  - `0x52` -- `REHOME_MAIN_JOINT_ENCODER_FLAG`
  - `0x53` -- `ZERO_IMU_FLAG`
  - `0x55` -- `CAPS_STOP_STREAM_LEG_STIFF_FLAG` (keep motors enabled)
  - `0x56` -- `INVERT_SHANK_THIGH_FLAG` (Fox Knee only)
  - `0x0B` -- `RESET_CONTROLLER_FLAG`
- **PDCP command `0x04`**: For knee devices, a 4-byte payload updates
  `SUBJECT_BUTTON_STATE` from the safety switch. For ankle devices, a 5-byte
  payload with sub-command `0x50` sets `ANKLE_OFFSET_FLAG`.

#### process_CAPS_Buzzer (CAPS10)

Extracts frequency from bytes 1-2 of the CAPS10 payload. If nonzero, stops any
active buzzer, then sets `CAPS_BUZZER_SCHEDULED_FLAG` so the main loop can call
the buzzer with the full frequency/volume/duration parameters extracted in
`caps_command_handler.buzzer_state`.

#### process_LOAD_CELL_DATA (LOAD_CELL1_DATA1, LOAD_CELL1_DATA2)

Delegates directly to `CanPdcpNidHandler.process_can_message` to decode 16-bit
3-channel loadcell frames into `sm_channels_current` slots 1-6.

#### process_FTS_Setting (FTS control NID)

Handles FTS-specific start/stop streaming commands. This is separate from
`process_CAPS_Setting` so that FTS start/stop does not trigger main actuator
stream start/stop.

- `0x50` with payload length 2: Sets `FTS_STREAM_FLAG = True`.
- `0x51` with payload length 3: Sets `FTS_STREAM_FLAG = False`.

FTS data NIDs (`can_out_fts_data1`, `can_out_fts_data2`) are sent in
`tick_caps` on even ticks when `FTS_STREAM_FLAG` is `True`.

### ACK Mechanism

When a received CAN message has the mode bit (`0x100`) set in its Standard ID,
`check_if_ack` records the original ID and sets `can_ack_flag` on the channel.

`can_ack` constructs a response with:

- The CAN ID equal to the received ID minus the mode bit (`received_id - 0x100`).
- Byte 0: original function code + `0x80`.
- Byte 1: status code (`1` = success, `>1` = error).
- Remaining bytes: echo of the original payload (up to 8 bytes total).

The response is enqueued via `can_sender.cpy_mess_into_can_send`.
`can_ack_all` iterates over CAPS1, CAPS5, and CAPS7 to acknowledge all pending
control messages in one call.

---

## CAN Sender (can_sender.py)

`canSender` provides a non-blocking, IRQ-safe CAN transmit path. Messages are
buffered during ISR context and drained by a hardware timer.

### Circular Buffer

- **Size**: 20 entries (`_CAN_SEND_BUFFER_LEN`).
- **Storage**: `can_send_buffer` (list of 20 `bytearray(8)`),
  `can_send_ids` (list of 20 CAN IDs), `can_send_len_idx` (list of 20 length
  indices).
- **Indexing**: `current_idx` is the next write position; `sent_idx` is the
  next read position. Both wrap modulo 20.
- **Overflow**: If `buffer_counter - send_counter` exceeds the buffer length,
  `reset_buffer` zeroes all indices.

### Enqueue: cpy_mess_into_can_send(data, did)

Copies 8 bytes from `data` into the buffer slot at `current_idx` using
`assembler_functions.cpy_can`, records the CAN ID and data length, increments
`buffer_counter`, and advances `current_idx`. Approximately 45 microseconds on
the F767.

For transmission, pre-computed `memoryview` slices (`can_send_buffer_mv`) allow
sending only the relevant number of bytes without allocating new buffers.

### Processor-Specific Send Routines

Both routines are invoked by a hardware timer callback and are gated by
`stop_CAN_send_flag` (initialized `True` at boot so no messages are sent until
streaming is started).

#### send_message_H7 (STM32H747)

1. Computes `num_mess` from the difference between `current_idx` and
   `sent_idx`.
2. Reads `FDCAN_TXFQS` to determine how many TX FIFO slots are free. If fewer
   free slots than messages, defers to the next timer tick.
3. Disables IRQs, sends all pending messages in a tight loop via `can.send`
   with `timeout=0`, then re-enables IRQs.

#### send_message_F7 (STM32F767)

1. Reads `CAN1_TSR` byte 3 to determine how many of the three hardware
   transmit mailboxes are empty (`0x1C` = all 3 free, `0x18`/`0x0C`/`0x14` =
   2 free, etc.).
2. Sends up to the minimum of available mailboxes and pending messages.
3. Each `can.send` call is wrapped in `pyb.disable_irq()` / `pyb.enable_irq()`
   to prevent ISR re-entrancy. Approximately 50-90 microseconds total.

---

## CAPS Command Handler (caps_command_handler.py)

`CapsCommandHandler` processes discrete commands that arrive from the CAPS CAN
message generator. These are dispatched from the main loop when the
corresponding flag (set by `process_CAPS_Message`) is detected. Each method
accepts a single unused argument (for uniform callback signature).

### Commands

| Method | Trigger Flag | Hex Code | Description |
|--------|-------------|----------|-------------|
| `rehome_encoder` | `REHOME_MAIN_JOINT_ENCODER_FLAG` | `0x52` | Calls `servo.set_joint_offset()` to rehome the absolute encoder. Plays a 400 Hz buzzer during the operation. Verifies the result falls within `MAIN_JOINT_HOMING_WINDOW`; plays ascending tones on success, descending tones on failure. Sets `MAIN_JOINT_ENCODER_HOMED_FLAG` on success. |
| `zero_imu` | `ZERO_IMU_FLAG` | `0x53` | Calls `imu.tare()` to zero the IMU (leg must be vertical). Clears CAPS9 and CAPS13 data buffers. Plays ascending confirmation tones. |
| `invert_shank_thigh` | `INVERT_SHANK_THIGH_FLAG` | `0x56` | Negates the sign of `arc_tan2_taylor_coefs[13]` to invert shank/thigh orientation. Fox Knee only. Clears CAPS9 data. |
| `ankle_offset` | `ANKLE_OFFSET_FLAG` | `0x04`+`0x50` | Adjusts `ANKLE_NEUTRAL_OFFSET` based on direction byte: `0x01` for plantarflexion (subtracts offset, clamped to `DEG_LIMIT_FLEX`), `0x02` for dorsiflexion (adds offset, clamped to `DEG_LIMIT_EXT`). Clears CAPS13 data. |
| `reset_controller` | `RESET_CONTROLLER_FLAG` | `0x0B` | Plays a short buzzer tone, then calls `pyb.hard_reset()` for a full hardware reset. |
| `buzzer_state` | `CAPS_BUZZER_SCHEDULED_FLAG` | `0x54` | Extracts frequency (bytes 1-2), volume (bytes 3-4), and duration (bytes 5-6) from the CAPS10 payload and calls `led_buzzer.play_buzzer()`. Clears CAPS10 data afterward. Cancels if frequency is zero. |

---

## See Also

- [ISR Reference](./isr-reference.md)
- [Data Dictionaries](../../documentation/data-dictionaries.md)
- [Motor Control](./motor-control.md)
- [Module Index](./module-index.md)
