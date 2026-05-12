# Sensor Drivers Reference

This document covers all sensor drivers in the ll_common package: the BNO085 IMU, FTS force/torque sensor, and LPS25HB barometric pressure sensor.

---

## BNO085 IMU (`BNO085.py`, 1243 lines)

### Overview

9-axis inertial measurement unit (accelerometer, gyroscope, magnetometer) with integrated sensor fusion. Communicates via SPI using the SHTP (Sensor Hub Transport Protocol) and SH2 (Sensor Hub v2) data format.

### Protocol Stack

| Layer | Name | Purpose |
|-------|------|---------|
| Application | SH2 | Data format: report types, feature configuration |
| Transport | SHTP | Data transmission: channels, sequence numbers, headers |
| Physical | SPI | Full-duplex, interrupt-driven data ready |

### SHTP Channel Identifiers

| Channel | ID | Purpose |
|---------|-----|---------|
| Executable | `0x1` | Device commands (reset, boot) |
| Sensor Hub Control | `0x2` | Configuration (set/get features, FRS read/write) |
| Input Sensor Reports | `0x3` | Normal sensor data |
| Wake Input Reports | `0x4` | Wake-triggered sensor data |
| Gyro Rotation Vector | `0x5` | Gyro-specific rotation vector |

### Available Report Types

| Report | ID | Data |
|--------|-----|------|
| Raw Accelerometer | `0x14` | 3-axis raw acceleration |
| Accelerometer | `0x01` | 3-axis calibrated acceleration |
| Gyroscope | `0x02` | 3-axis angular velocity |
| Magnetometer | `0x03` | 3-axis magnetic field |
| Rotation Vector | `0x05` | Quaternion (i, j, k, real) |
| Game Rotation Vector | `0x08` | Quaternion without magnetometer |
| Pressure | `0x0A` | Barometric pressure (via external LPS25HB) |
| Gravity | `0x06` | 3-axis gravity vector |
| Linear Acceleration | `0x04` | Acceleration minus gravity |

### Key Commands

| Command | ID | Purpose |
|---------|-----|---------|
| Tare | `0x03` | Zero orientation reference |
| Initialize | `0x04` | Reset sensor fusion |
| Save DCD | `0x06` | Save dynamic calibration data |
| Motion Engine Calibrate | `0x07` | Trigger calibration |

### ISR Integration

The IMU is read at 50 Hz via the `tick_imu` ISR in `hybrid_leg.py`:

1. `assembler_functions.imu_isr(imu_isr_addresses)` — assembly-optimized SPI exchange reads all sensor data
2. Data stored in pre-allocated arrays referenced by memory address
3. IMU data sent to CAPS via `can_out_mes_imu1` and `can_out_mes_imu2`

### Q-Point Fixed-Point Conversion

BNO085 reports values as fixed-point integers with a Q-point exponent. Conversion to float:

```
float_value = raw_value × 2^(-Q)
```

Common Q-points: Rotation Vector Q=14, Accelerometer Q=8, Gyroscope Q=9.

The driver provides `q_to_float_32(raw, q)` for this conversion.

### Key Methods

| Method | Description |
|--------|-------------|
| `tare()` | Zero orientation to current position |
| `set_orientation()` | Configure reference frame |
| `send_receive_cyclic()` | Full-duplex SPI exchange (ISR-safe) |
| `q_to_float_32(raw, q)` | Fixed-point to float conversion |

---

## Force/Torque Sensor (`fts.py`, 810 lines)

### Overview

Measures vertical force (Fz) and moment about the y-axis (My) using two DRV5055-Q1 Hall effect sensors and an ADS1292 dual-channel 24-bit ADC. Uses a dual cantilever beam design.

### Data Flow

```
Hall Effect Sensors (DRV5055-Q1)
         |
         v
ADS1292 ADC (24-bit voltage, SPI)
         |
         v
Voltage Offset: v1 = V_ch1 - V0_ch1, v2 = V_ch2 - V0_ch2
         |
         v
Polynomial Evaluation:
    Fz = c0*v1 + c1*v2 + c2*v1² + c3*v2² + c4*v1*v2 + c5
    My = c0*v1 + c1*v2 + c2*v1² + c3*v2² + c4*v1*v2 + c5
         |
         v
Force (N) and Moment (Nm)
```

### Calibration

**Calibration coefficient order:** `[v1, v2, v1², v2², v1*v2, intercept]`

For degree-1 calibration, set quadratic/cross-term coefficients to 0.

| Step | Method | Description |
|------|--------|-------------|
| 1 | `zero_calibration()` | Capture V0 reference voltages in unloaded state |
| 2 | `set_calibration(fz_coeffs, my_coeffs)` | Set polynomial coefficients from calibration notebook |

V0 values can also be set directly: `sensor.v0[0] = 0.663443`

### ADS1292 ADC Configuration

| Parameter | Value |
|-----------|-------|
| Resolution | 24-bit |
| Channels | 2 (differential) |
| Sample rate | 125–8000 SPS (configurable) |
| Communication | SPI |
| Data ready | Interrupt pin (DRDY) |

### Key Methods

| Method | Description |
|--------|-------------|
| `start_conversions()` | Begin ISR-based continuous streaming |
| `zero_calibration()` | Capture V0 reference voltages (unloaded) |
| `set_calibration(fz_coeffs, my_coeffs)` | Set polynomial coefficients |
| `get_force_moment()` | Calculate Fz and My from current readings |
| `get_magnetic_field()` | Hall sensor diagnostic (linearized field value) |

### SPI Bus Sharing (vB_3_0)

On the vB_3_0 board the FTS ADS1292 shares SPI bus 1 with the BNO085 IMU. Two
safeguards in `hardware_manager.set_fts_variables()` prevent bus contention:

1. **Initialization**: The BNO085 ExtInt is disabled before ADS1292
   configuration and re-enabled after `start_conversions()`. This prevents
   the BNO085 ISR from asserting its CS and driving MISO during register
   writes.
2. **Runtime**: The FTS DRDY ExtInt is set to the same NVIC priority as the
   BNO085 ExtInt (priority 5). At equal priority neither ISR can preempt
   the other, so their SPI transactions never overlap.

### Output Integration

The on-board FTS is polled at 500 Hz inside `isr_joint_controller` (Fox Knee only). When `fts_sensor` is present, the ISR calls `get_force_moment()` and writes the results to:

- `PARAM_DICT['FTS_FZ']` — Force in Z direction (N)
- `PARAM_DICT['FTS_MY']` — Moment about Y axis (Nm)

### CAN Streaming

FTS data is streamed over CAN as a drop-in replacement for the external load
cell PDCP structure. Two 16-bit, 3-channel data NIDs are sent on even ticks in
`tick_caps` (125 Hz effective rate) when `FLAG_DICT['FTS_STREAM_FLAG']` is True:

- **NID 0x045** (`can_out_fts_data1`): [zero, zero, Fz] — Fz in channel 3
  position. Force gain = 6553.5 / 500 = 13.107.
- **NID 0x046** (`can_out_fts_data2`): [zero, My, zero] — My in channel 5
  position. Moment gain = 6553.5 / 40 = 163.8375.

Streaming is controlled by start (0x50) and stop (0x51) commands on the FTS
control NID (0x044), handled by `can_class.process_FTS_Setting()`. The FTS
stream flag is independent of the main actuator `STREAM_FLAG`.

### Transient Rejection Filter

The FTS ISR includes a two-layer filter that rejects corrupted SPI reads. On the vB_3_0 board the ADS1292 shares SPI bus 1 with the BNO085 IMU, and occasional single-sample transients with physically impossible values have been observed (~once per 5 minutes).

**Layer 1 — Status word validation (always active):** The ADS1292 status byte upper nibble must be `0xC` (device header bits `1100`). Frames where the SPI read returned shifted or zeroed bytes typically fail this check.

**Layer 2 — Rate-of-change guard (configurable):** When `transient_reject_threshold > 0`, the ISR compares each channel's new raw count against the previous valid reading. If either delta exceeds the threshold, the sample is rejected. The filter arms after the first valid sample passes the status word check.

Rejected samples hold the previous valid reading — a single 2 ms gap is unnoticeable in the control loop.

| Parameter | Location | Value |
|-----------|----------|-------|
| `transient_reject_threshold` | `FTS.__init__` | Constructor parameter, default 0 (disabled) |
| `FTS_TRANSIENT_REJECT_THRESHOLD` | `const_dict.py` (Fox Knee) | 250,000 raw counts |

Diagnostic attribute: `fts_sensor.rejected_count` — incremented on each rejected sample, accessible from the REPL.

---

## LPS25HB Barometric Pressure Sensor (`LPS25HB.py`, 29 lines)

### Overview

I2C barometric pressure sensor used for altitude estimation. Minimal driver with single read method.

### Configuration

| Parameter | Value |
|-----------|-------|
| I2C Address | `0x5C` |
| Sample Rate | 25 Hz (internal, max) |
| FIFO | Enabled |
| Filtering | Moving average of 32 samples |
| Register `0x20` | `0xC0` (active mode, 25 Hz) |
| Register `0x21` | `0x40` (FIFO enable) |
| Register `0x2E` | `0xDF` (moving average 32) |

### Key Methods

| Method | Description |
|--------|-------------|
| `readPress()` | Returns raw 24-bit pressure value (3 bytes combined) |

### Altitude Calculation

Altitude is computed in `hybrid_leg.tick_imu()`:

```python
pressure = q_to_float_32(raw_pressure, 20)
altitude = 0.0044 * pressure² - 17.175 * pressure + 12877.0
```

Results stored in:
- `PARAM_DICT['BAR_BYTE_VALS'][0]` — Startup barometer reading
- `PARAM_DICT['BAR_BYTE_VALS'][1]` — Current barometer reading
- `PARAM_DICT['ALTITUDE']` — Computed altitude (meters)

---

## See Also

- [ISR Reference](./isr-reference.md) — `tick_imu` ISR, IMU assembly routine
- [CAN and CAPS Protocol](./caps-and-can.md) — IMU and FTS CAN output messages
- [Data Dictionaries](../../documentation/data-dictionaries.md) — Sensor-related PARAM_DICT keys
