# ll_common Module Index

This is the navigational entry point for the `ll_common` shared library. All modules in this package are listed below with their category, line count, and purpose. For application-level documentation, see the [main repo documentation](../../documentation/).

## Module Dependency Graph

```
hybrid_leg.py (orchestrator)
├── hardware_manager.py
│   ├── routing.py
│   ├── novanta.py ──► novanta_constants.py
│   ├── BNO085.py
│   ├── fts.py
│   ├── fts_parameters.py
│   ├── upy_spi.py
│   ├── can_sender.py ──► assembler_functions.py
│   └── LPS25HB.py
├── caps_manager.py
│   └── CAN_PDCP_Streaming_Message.py ──► assembler_functions.py
├── caps_command_handler.py
├── can_class.py ──► assembler_functions.py
├── controller.py
├── state_machine.py ──► sm_assembler_functions.py
├── error_flag.py
├── led_buzzer_handler.py
├── bleComms.py ──► ble_advertising.py
├── upy_float.py
├── logger.py
├── mode_switch.py
├── nvic.py
├── decorators.py
├── const_dict.py ──► novanta_constants.py
├── param_dict.py
└── flag_dict.py
```

## Module Quick Reference

| Module | Lines | Category | Description |
|--------|------:|----------|-------------|
| `hybrid_leg.py` | 1128 | Core | Top-level orchestrator; owns ISRs (`isr_joint_controller`, `tick_imu`, `tick_caps`), initialization chain, kinematic calculations |
| `novanta.py` | 2064 | Motor | Everest servo driver; SPI communication, SDO/PDO protocol, CiA402 state machine, CRC verification |
| `novanta_constants.py` | 628 | Motor | Everest register addresses, state codes, data type constants, control word values |
| `controller.py` | 122 | Motor | PD impedance controller; `output = K*error - B*velocity - D*d(error)/dt + T_bias` |
| `can_class.py` | 565 | CAN | CAN bus manager; message filtering per device, FIFO callbacks, function-dict dispatch, ACK mechanism |
| `caps_manager.py` | 386 | CAN | CAPS NID definitions; PDCP input/output channel setup, `CAN_INPUT_DICT` construction |
| `CAN_PDCP_Streaming_Message.py` | 179 | CAN | PDCP message format handler; scaling, sequence numbers, 12-bit/16-bit channel packing |
| `can_sender.py` | 205 | CAN | Non-blocking 20-entry circular CAN TX buffer; processor-specific send with IRQ safety |
| `caps_command_handler.py` | 107 | CAN | CAPS command dispatch (rehome encoder, zero IMU, ankle offset, reset controller, buzzer) |
| `BNO085.py` | 1243 | Sensor | BNO085 IMU driver; SPI + SHTP protocol, quaternion/Euler/gyro/accel reports, interrupt-driven |
| `fts.py` | 810 | Sensor | Force/torque sensor; ADS1292 24-bit ADC, DRV5055 Hall sensors, polynomial calibration |
| `fts_parameters.py` | 58 | Sensor | Per-unit FTS parameters: calibration coefficients, zero voltages, and PDCP NID set |
| `LPS25HB.py` | 29 | Sensor | Barometric pressure sensor; I2C driver, `readPress()` method |
| `state_machine.py` | 162 | Control | Loads and executes `.psm` binary files from CAPS; timer-driven state transitions |
| `sm_assembler_functions.py` | 1246 | Control | ARM assembly optimized state machine operations (input update, event check, state output) |
| `assembler_functions.py` | 1000 | Control | Optimized math routines (trig, CAN parsing, joint velocity, polynomial evaluation) |
| `routing.py` | 981 | HAL | Board-specific pin assignments; `Protocol`/`Component` classes; 3 board variants |
| `hardware_manager.py` | 682 | HAL | Component factory; instantiates all hardware based on routing config and device type |
| `mode_switch.py` | 112 | HAL | ADC-based device type detection; maps switch positions to device IDs |
| `bleComms.py` | 165 | BLE | BLE GATT interface; Nordic UART Service emulation, connect/disconnect/notify |
| `ble_advertising.py` | 83 | BLE | BLE advertisement packet formatting |
| `upy_spi.py` | 677 | Utility | ISR-safe SPI driver; viper-compiled, direct STM32 register access, processor-specific |
| `upy_float.py` | 166 | Utility | Float wrapper with `stm.mem32` access for ISR-safe operations; `upyFloat` class |
| `nvic.py` | 40 | Utility | NVIC interrupt priority management; `dump_nvic()`, `nvic_set_prio()` |
| `decorators.py` | 51 | Utility | `@singleton` class decorator used by `Board` in config.py |
| `logger.py` | 46 | Utility | Binary data logger; writes `array.array` to file for state machine channel recording |
| `const_dict.py` | 490 | Data | Immutable system constants; Taylor coefficients, kinematics, LED colors, Everest config |
| `param_dict.py` | 72 | Data | Mutable runtime parameters; joint state, motor commands, battery, timing, sensor values |
| `flag_dict.py` | 34 | Data | Boolean state flags; stream control, motor state, watchdog, CAPS command triggers |
| `error_flag.py` | 180 | Data | 12 error conditions with CAPS signal levels and buzzer alerts |
| `led_buzzer_handler.py` | 253 | UI | LED color patterns and buzzer frequency/duration control for status indication |
| `__init__.py` | 0 | Package | Empty package marker |

## Documentation Map

### Library Documentation (this directory)
- [ISR Reference](./isr-reference.md) — All 5 timer ISRs: execution flow, timing budgets, priorities
- [Motor Control](./motor-control.md) — Everest servo, impedance controller, transmission ratios
- [CAN and CAPS Protocol](./caps-and-can.md) — CAN bus, PDCP NID map, message formats, dispatch
- [Sensor Drivers](./sensor-drivers.md) — BNO085 IMU, FTS force/torque, LPS25HB barometer
- [Hardware Manager](./hardware-manager.md) — Routing, component factory, board variants
- [State Machine](./state-machine.md) — .psm loader, assembly execution, CAPS integration
- [Utility Modules](./utility-modules.md) — SPI, float wrapper, NVIC, assembler functions, BLE, LED/buzzer

### Application Documentation (main repo)
- [Boot Sequence](../../documentation/boot-sequence.md) — Power-on through steady-state
- [Device Configuration](../../documentation/device-configuration.md) — Device types, parameters, routing
- [Main Loop](../../documentation/main-loop.md) — MainLoop class, timer_loop ISR, watchdog, streaming
- [Data Dictionaries](../../documentation/data-dictionaries.md) — FLAG_DICT, PARAM_DICT, CONST_DICT, error flags
