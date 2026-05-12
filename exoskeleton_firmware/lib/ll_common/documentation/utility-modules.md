# Utility Modules Reference

This document covers the supporting utility modules in the ll_common package.

---

## upy_spi.py — ISR-Safe SPI Driver (677 lines)

### Overview

Extends `pyb.SPI` with ISR-safe methods that bypass MicroPython's SPI layer using direct STM32 register manipulation. Viper-compiled for maximum performance.

### Class: `upySPI`

Provides processor-specific SPI methods:

**STM32H747 (Portenta):**
| Method | Description |
|--------|-------------|
| `readinto_portenta(buf)` | Read SPI data into buffer |
| `write_portenta(buf)` | Write buffer to SPI |
| `write_readinto_portenta(wbuf, rbuf)` | Full-duplex exchange |

**STM32F767 (Pyboard):**
| Method | Description |
|--------|-------------|
| `readinto_pyboard(buf)` | Read SPI data into buffer |
| `write_pyboard(buf)` | Write buffer to SPI |
| `write_readinto_pyboard(wbuf, rbuf)` | Full-duplex exchange |

### Key Features
- Direct STM32 SPI peripheral register access (DR, SR, CR1)
- Handles MODFault soft reset recovery
- Viper-compiled (`@micropython.viper`) for deterministic timing
- Used by Everest servo and BNO085 IMU drivers

---

## upy_float.py — Float Memory Wrapper (166 lines)

### Overview

`upyFloat` wraps a single float value in `array.array('f', [value])`, providing direct memory access via `stm.mem32` for ISR-safe operations without heap allocation.

### Class: `upyFloat`

| Property | Type | Decorator | Description |
|----------|------|-----------|-------------|
| `val` | `float` | `@micropython.native` | Get/set the float value |
| `adr` | `int` | `@micropython.native` | Memory address of the raw bytes |
| `raw_bytearray` | `bytearray(4)` | `@micropython.native` | Raw byte representation |
| `raw_bytes_as_int` | `int` | `@micropython.native` | Raw bytes as integer |

### Usage Patterns

```python
motor_velocity = upyFloat(0.0)

# ISR-safe write (no heap allocation)
motor_velocity.val = servo.channel_poci.actual_velocity * 360.0

# Pass address to assembly function
assembler_functions.compute_joint_velocity(
    PARAM_DICT['LARGE_MOTOR_TRANSMISSION_RATIO'],
    motor_velocity.adr,    # memory address
    PARAM_DICT['JOINT_SPEED_ARRAY']
)
```

Used for `motor_velocity` in `hybrid_leg.py` and IMU channel data.

---

## assembler_functions.py — Optimized Math (1000 lines)

### Overview

ARM assembly and viper-compiled math routines for performance-critical operations. Called from ISRs where standard Python would be too slow.

### Key Functions

| Function | Description | Called By |
|----------|-------------|----------|
| `compute_joint_velocity(transmission, motor_vel_adr, output)` | Joint speed from motor velocity × transmission | `isr_joint_controller` |
| `calculate_polynomial(output, input, coefficients, length)` | General polynomial evaluation | `isr_joint_controller` (ankle angle, transmission) |
| `imu_isr(imu_isr_addresses)` | Complete IMU SPI exchange in optimized code | `tick_imu` |
| `fast_caps_read_12bit_4chans(data, channel_array, gains)` | Decode 12-bit CAN message to 4 float channels | CAN callbacks |
| `fast_caps_read_16bit(data, channel_array, gains)` | Decode 16-bit CAN message to 3 float channels | CAN callbacks |
| `fw_CAPS_u12bit(float_data, output_buf)` | Encode 4 float channels to 12-bit CAN message | `tick_caps` |
| `fw_CAPS_u16bit(data, outmap, gain_map)` | Encode 3 float channels to 16-bit CAN message | `tick_caps` (FTS) |
| `fob_value_calc(fob_array, calc_array)` | Scale key fob ADC value | CAN callback |
| `cpy_can(source, dest, length)` | Fast CAN buffer copy | `can_sender` |
| `mov_from_to(source, dest, count)` | Memory-to-memory float copy with stride | Various |
| `ass_log(input_adr, output_adr, coefs)` | Taylor series natural log | Kinematics |

---

## nvic.py — Interrupt Priority Management (40 lines)

### Overview

Direct manipulation of the ARM Cortex-M NVIC (Nested Vectored Interrupt Controller) for setting interrupt priorities.

### Memory Map

| Register | Address | Description |
|----------|---------|-------------|
| SCS | `0xE000E000` | System Control Space base |
| NVIC | SCS + `0x0100` | NVIC base |
| SCB | SCS + `0x0D00` | System Control Block |
| NVIC_PRIO | NVIC + `0x300` | Priority registers |
| SCB_SHP | SCB + `0x18` | System handler priority registers |

### Functions

| Function | Description |
|----------|-------------|
| `dump_nvic()` | Print all non-zero interrupt priorities (system + regular) |
| `nvic_set_prio(irq, prio)` | Set priority (0–15) for IRQ number. Negative IRQs for system handlers. |

Priority is stored in upper 4 bits of the priority register byte (`prio << 4`).

---

## decorators.py — Singleton Pattern (51 lines)

### `@singleton` Decorator

Ensures only one instance of a decorated class exists across all imports. Used by `Board` class in `config.py`.

```python
@decorators.singleton
class Board(object):
    def __init__(self):
        ...
```

Maintains a per-class instance dictionary. Subsequent calls to `Board()` return the existing instance.

---

## logger.py — Binary Data Logger (46 lines)

### Class: `Logger`

Records `array.array` data to file in binary format.

| Method | Description |
|--------|-------------|
| `__init__(filename, ftype)` | Open file. `ftype=1` for int, `ftype=2` for float |
| `writeData(data_array)` | Write array contents to open file |
| `close()` | Close file |

Used for state machine channel logging (`log_sm_channels` in `hybrid_leg.py`).

---

## bleComms.py — BLE GATT Interface (165 lines)

### Overview

Bluetooth Low Energy communication using Nordic UART Service (NUS) emulation.

### UUIDs

| UUID | Name | Direction |
|------|------|-----------|
| `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` | NUS Service | — |
| `6E400002-...` | RX Characteristic | Write (phone → device) |
| `6E400003-...` | TX Characteristic | Notify (device → phone) |

### Class: `BLEComms`

| Method | Description |
|--------|-------------|
| `__init__(ble, name)` | Setup BLE service, start advertising with device name |
| `_irq(event, data)` | IRQ handler for CONNECT, DISCONNECT, WRITE events |
| `sendMessage(data)` | Send notification to connected BLE client |
| `getMessage()` | Retrieve received message |
| `checkConnectionStatus()` | Returns `True` if connected |
| `checkMessageFlag()` | Returns `True` if message pending |
| `disconnectBLE()` | Graceful disconnect with retry |

### BLE Message Protocol

Messages from phone app are parsed in `hybrid_leg.parse_ble_message()`:
- `0x01, 0x00` — Zero load cell + tare IMU
- `0x01, 0x01` — Reset WiFi
- `0x01, 0x03` — Rehome encoder
- `0x01, 0x04` — Zero IMU
- `0x02, 0x01–0x04` — Set active phone app tab (1–4)

---

## ble_advertising.py — BLE Advertising (83 lines)

### Function: `advertising_payload()`

Formats BLE advertisement data per the BLE specification. Includes device name for mobile app discovery.

---

## led_buzzer_handler.py — Status Indicators (253 lines)

### Class: `LedBuzzerHandler`

### LED Status Logic

| Condition | Pyboard LED | External LED |
|-----------|-------------|--------------|
| Not streaming | Blue blink | Depends on errors/battery |
| Motor active | Yellow solid | — |
| Braking active | Turquoise solid | — |
| Streaming, idle | Green blink | — |
| Error active | — | Magenta |
| Low battery | — | Red/Green blink |
| Error + low battery | — | Magenta/Red fast blink |

### Buzzer Control

| Method | Description |
|--------|-------------|
| `play_buzzer(freq, msec, duty_cycle, device_to_buzz)` | Play tone |
| `startup_led_sequence()` | Boot animation |

Constraints:
- Duration cap: 2000 ms
- Duty cycle cap: 90%
- `device_to_buzz`: `"PRIMARY"` (default) or `"ALL"` (all devices via CAN)

### Critical Battery Buzzer

Decreasing frequency sweep per 0.5V drop below threshold. Starting frequency: 800 Hz, decreasing as voltage drops.

### Error Buzzer

800 Hz, 100 ms tone every `ERROR_BUZZ_INTERVAL` (200 ms) when any buzzable error flag is active.

---

## See Also

- [Module Index](./module-index.md) — Complete module listing
- [ISR Reference](./isr-reference.md) — How utility modules are used in ISRs
- [Data Dictionaries](../../documentation/data-dictionaries.md) — LED color constants in CONST_DICT
