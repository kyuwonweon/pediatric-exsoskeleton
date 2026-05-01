# MicroPython Lower Limb Common Library

Shared driver and utility library for the hybrid leg prosthetic control system. This package provides motor control, sensor drivers, CAN communication, state machine execution, and hardware abstraction for STM32-based MicroPython controllers.

## Module Categories

| Category | Key Modules |
|----------|-------------|
| Orchestration | `hybrid_leg.py` -- main device orchestrator and ISR host |
| Motor Control | `novanta.py` (Everest servo), `controller.py` (impedance) |
| Communication | `can_class.py`, `can_sender.py`, `caps_manager.py`, `CAN_PDCP_Streaming_Message.py` |
| Sensors | `BNO085.py` (IMU), `fts.py` (force/torque), `LPS25HB.py` (barometer) |
| Hardware | `hardware_manager.py`, `routing.py`, `mode_switch.py` |
| State Machine | `state_machine.py`, `sm_assembler_functions.py` |
| Utilities | `assembler_functions.py`, `upy_spi.py`, `upy_float.py`, `nvic.py` |

## Documentation

| Document | Description |
|----------|-------------|
| [Module Index](documentation/module-index.md) | Master index of all modules with dependency graph |
| [ISR Reference](documentation/isr-reference.md) | All timer ISRs: execution flow, timing, device branching |
| [Motor Control](documentation/motor-control.md) | Everest servo, impedance controller, CiA402 states |
| [CAN and CAPS Protocol](documentation/caps-and-can.md) | PDCP NID map, message formats, CAN callbacks |
| [Sensor Drivers](documentation/sensor-drivers.md) | BNO085 IMU, FTS force/torque, LPS25HB barometer |
| [Hardware Manager](documentation/hardware-manager.md) | Routing classes, component factory, bus sharing |
| [State Machine](documentation/state-machine.md) | .psm binary format, assembly functions, CAPS integration |
| [Utility Modules](documentation/utility-modules.md) | SPI driver, float wrapper, assembler functions, BLE, LEDs |

## Development History

For details on what was changed for the latest release, please see
the [Changelog.md](Changelog.md).
