# Hardware Manager

The hardware manager is the runtime factory responsible for instantiating every
peripheral driver on the lower-limb controller. It sits at the top of a
three-layer hardware abstraction that separates physical pin assignments,
application-level feature policy, and actual driver construction.

## Architecture

Hardware abstraction is split across three files, each with a single
responsibility:

| Layer | File | Responsibility |
|-------|------|----------------|
| 1 -- Hardware truth | `routing.py` | Board-specific pin assignments, bus numbers, and default protocols. One class per PCB revision. |
| 2 -- Application policy | `config.py` (`Board` class) | Selects the routing class, enables or disables individual components, and may override default protocols. |
| 3 -- Runtime factory | `hardware_manager.py` | Reads the component flags and pin routes, then creates driver instances and attaches them to the parent `HybridLeg` object. |

Data flows downward at startup:

```
config.Board()                         # policy: which board, which features
    --> routing.UPythonLLCtrl_vB_x_x() # truth: pin map for that board
        --> HardwareManager             # factory: build drivers from the above
```

`config.Board().get_components()` returns the singleton routing instance with
the appropriate components enabled. `HardwareManager` iterates over its setup
methods, and each method checks `components.<Name>.is_enabled()` before
creating any driver objects.

## Routing Classes (`routing.py`)

### `Protocol` Enum

Defines all communication protocols that a component may use. The integer
values are used as identifiers and are assigned to each component's `protocol`
property in the routing class constructor.

| Name | Value | Description |
|------|-------|-------------|
| `ANALOG` | 1 | Component is an analog device (input or output). |
| `CAN` | 2 | Component communicates over CAN. |
| `GPIO` | 3 | Component utilizes generic GPIO pin(s). |
| `I2C` | 4 | Component communicates over I2C. |
| `RS_232` | 5 | Component communicates over RS-232. |
| `RS_485` | 6 | Component communicates over RS-485. |
| `SK6812` | 7 | LED of type SK6812; requires bit-banged protocol. |
| `SPI` | 8 | Component communicates over SPI. |
| `UART` | 9 | Component communicates over UART. |

Value `0` is reserved for "not set".

### `Component` Base Class

Every hardware component inherits from `Component`. The base class provides:

- **`enable()`** -- Sets the internal `_active` flag to `True`.
- **`disable()`** -- Sets the internal `_active` flag to `False`.
- **`is_enabled()`** -- Returns the current `_active` flag.
- **`protocol`** (property) -- Getter/setter backed by `_protocol`. Accepts
  a `Protocol` member as its value.

All components start disabled (`_active = False`) and with `_protocol = None`.
The routing class constructor assigns a default protocol for each component;
the `config.py` `Board` class is responsible for calling `enable()` on the
components the application needs.

### Board Variant Classes

Each class is decorated with `@decorators.singleton` (see
[Singleton Pattern](#singleton-pattern) below) and maps every hardware feature
to its physical pin assignments for a specific PCB revision.

#### `UPythonLLCtrl_vB_1_1` -- Pyboard SF6 (STM32F767)

Board identity: `upython-ll-ctrl vB.1.1`

Target platform for the original **Hybrid Knee** device. Pin references use
the Pyboard W-bus, X-pin, and Y-pin notation (e.g. `W54`, `X9`, `Y11`).

Components defined:

| Component | Default Protocol | Key Pins / Buses |
|-----------|-----------------|-------------------|
| `AuxiliaryAnalogIn` | ANALOG | Channels on W19, W17, W15, W11 |
| `BrakingCircuit` | GPIO | FET enable: W54, PWM: W52 (inverted) |
| `Buzzer` | GPIO | Line out: W19 |
| `Everest` | SPI | SPI bus 1, CS: W7, STO1: W49, STO2: W45, IRQ: W24, Reset: W22 |
| `ESCON` | GPIO | Enable: W73, Current: W63, Direction: W74, Fault: W71 |
| `ExternalComm` | CAN | CAN bus 1, RX: X9, TX: X10 |
| `ExternalLEDs` | GPIO | Blue: W27, Green: W67, Red: W65 |
| `IMU` | SPI | SPI bus 3, CS: W16, Reset: W25, Data ready: W33 |
| `InternalLEDs` | GPIO | IDs: 1 (red), 2 (green), 3 (blue) |
| `ModeSelectionSwitch` | ANALOG | Line in: W53 |
| `QwiicConnector` | I2C | I2C bus 2, SCL: Y9, SDA: Y10 |
| `SafetySwitch` | ANALOG | Line in: Y11 |
| `TimingOutput` | -- | Line out: W47 |

#### `UPythonLLCtrl_vB_2_0` -- Portenta H7 v2 (Hybrid Knee)

Board identity: `upython-ll-ctrl vB.2.0`

Target platform for the **Hybrid Knee** on the Portenta H7. Pin references use
the high-density connector notation (e.g. `J2_65`, `J1_51`).

Components defined:

| Component | Default Protocol | Key Pins / Buses |
|-----------|-----------------|-------------------|
| `BrakingCircuit` | GPIO | FET enable: J2_65, PWM: J2_60 (non-inverted) |
| `Buzzer` | GPIO | Line out: J2_61 |
| `ESCON` | GPIO | Enable: J2_6, Current: J2_73, Direction: J2_4, Fault: J2_8 |
| `Everest` | SPI | SPI bus 2, CS: J2_36, STO1: J2_68, STO2: J2_67, IRQ: J2_54, Reset: J2_52 |
| `ExternalComm` | CAN | CAN bus 1, RX: J1_51, TX: J1_49 |
| `ExternalLEDs` | GPIO | Blue: J2_14, Green: J2_10, Red: J2_12 |
| `IMU` | SPI | SPI bus 1, CS: J2_46, Reset: J2_16, Data ready: J2_62 |
| `InternalLEDs` | GPIO | LED_RED, LED_GREEN, LED_BLUE |
| `ModeSelectionSwitch` | ANALOG | Line in: J2_79 |
| `QwiicConnector` | I2C | I2C bus 3, SCL: J1_46, SDA: J1_44 |
| `TimingOutput` | -- | Line out: J1_33 |

#### `UPythonLLCtrl_vB_3_0` -- Portenta H7 v3 (Fox Knee)

Board identity: `upython-ll-ctrl vB.3.0`

Target platform for the **Fox Knee** alpha. Shares the Portenta H7
high-density connector pin notation with vB_2_0 but introduces new
components and removes some legacy ones.

Components defined:

| Component | Default Protocol | Key Pins / Buses |
|-----------|-----------------|-------------------|
| `BrakingCircuit` | GPIO | FET enable: J2_65, PWM: J2_60 (non-inverted) |
| `Buzzer` | GPIO | Line out: J2_61 |
| `ESCON` | -- | Dummy entry (no ESCON support on this board) |
| `Everest` | SPI | SPI bus 2, CS: J2_36, STO1: J2_68, STO2: J2_67, IRQ: J2_54, Reset: J2_52 |
| `ExternalComm` | CAN | CAN bus 1, RX: J1_51, TX: J1_49 |
| `ExternalLEDs` | GPIO | Blue: J2_14, Green: J2_10, Red: J2_12 |
| `ForceTorqueSensor` | -- | SPI bus 1, CS: J2_4, Data ready: J2_8, Power reset: J1_46, Start: J1_44 |
| `IMU` | SPI | SPI bus 1, CS: J2_46, Reset: J2_16, Data ready: J2_62 |
| `InternalLEDs` | GPIO | LED_RED, LED_GREEN, LED_BLUE |
| `ModeSelectionSwitch` | ANALOG | Line in: J2_79, Variant in: J2_77 |
| `PeripheralPower` | GPIO | External SPI power: J2_28 |
| `QwiicConnector` | -- | Dummy entry (no Qwiic on this board) |
| `TimingOutput` | -- | Line out: J1_33 |

Notable differences from vB_2_0:
- Adds `ForceTorqueSensor` (SPI bus 1) and `PeripheralPower` components.
- `ESCON` is a stub class with no pin assignments.
- `QwiicConnector` is a stub class; the I2C pins previously used for Qwiic are
  reassigned to the force torque sensor (`power_reset`, `start`).
- `ModeSelectionSwitch` gains a `variant_in` GPIO line (dip switch replacing
  the 10-position rotary switch).

## HardwareManager (`hardware_manager.py`)

### Class Overview

```python
class HardwareManager:
    def __init__(self, parent):
        self.parent = parent
        self.parent.i2c_instances = [None] * 3   # index 0..2
        self.parent.spi_instances = [None] * 5   # index 0..4
```

`parent` is the main `HybridLeg` orchestrator instance. Every driver, bus
handle, timer, and pin object the manager creates is attached as an attribute
of `parent`, making them accessible throughout the application.

The `i2c_instances` and `spi_instances` arrays are pre-allocated to track
shared bus objects. Indices are 1-based by convention (bus 1 lives at index 1;
index 0 is unused).

### `initialize_all_hardware()` Sequence

The single public entry point calls setup methods in a fixed order:

| # | Method | What It Initializes |
|---|--------|---------------------|
| 1 | `set_timers()` | pyb.Timer objects for IMU, state machine, braking PWM, CAPS send, buzzer PWM, CAN external, main loop, and motor control. Assigns NVIC interrupt priorities. |
| 2 | `set_auxiliary_comm_variables()` | Reserved; raises an exception if enabled (no implementation yet). |
| 3 | `set_barometer_variables()` | LPS25HB barometer driver over I2C. Creates shared I2C bus instance if needed. |
| 4 | `set_braking_circuit_variables()` | PWM channel for dynamic braking and the FET enable GPIO. Selects `PWM_INVERTED` for STM32F767 or normal `PWM` for STM32H747 based on the routing `PWM.invert` flag. |
| 5 | `set_buzzer_variables()` | PWM channel for the external buzzer. |
| 6 | `set_everest_variables()` | SPI bus for the Novanta Everest motor controller (50 MHz on H747, 30 MHz on F767). Creates the `Everest` driver instance, enables STO lines, transitions the servo to cyclic mode, and instantiates the impedance `Controller`. |
| 7 | `set_external_comm_variables()` | CAN bus initialization. Uses `sample_point`/`baudrate` on H747 or `prescaler`/`sjw`/`bs1`/`bs2` on F767 for 1 Mbit/s. Creates `canSender` wrapper. |
| 8 | `set_external_led_variables()` | GPIO pins (OUT, PULL_DOWN) for blue, green, and red external LEDs. |
| 9 | `set_imu_variables()` | SPI bus for the BNO085 IMU (12 MHz on H747, 4 MHz on F767). Allocates IMU data arrays, Taylor coefficient array for arctan2 angle calculation, and configures report types (rotation vector, linear acceleration, calibrated gyroscope, pressure). |
| 10 | `set_joint_encoder_variables()` | ADC (ANALOG protocol) or I2C (legacy) for the joint angle encoder. |
| 11 | `set_qwiic_connector_variables()` | I2C bus for the Qwiic connector. |
| 12 | `set_timing_output_variables()` | GPIO pin (OUT_PP, PULL_UP) for code timing instrumentation. |

The method `set_auxiliary_analog_in_variables()` exists in the class but is
commented out of the `setup_methods` list.

### Component Instantiation Pattern

Every `set_*_variables()` method follows the same guard-check-create pattern:

```python
def set_<component>_variables(self):
    if self.parent.components.<Component>.is_enabled():
        # 1. Read protocol and pin info from components
        # 2. Create or reuse a shared bus instance
        # 3. Instantiate the driver, passing bus + pins
        # 4. Attach the driver to self.parent
    else:
        print("  ... Skipping '<Component>'")
```

Most methods wrap the body in a `try/except` to allow graceful degradation
when a component attribute does not exist on the active routing class (e.g.
`Barometer` or `JointEncoder` may not be present on every board variant).

### SPI / I2C Bus Sharing

Buses are tracked in `parent.spi_instances[1..4]` and
`parent.i2c_instances[1..2]`. Before creating a new bus, each setup method
checks whether the instance already exists:

```python
bus_num = self.parent.components.Barometer.I2C.bus_number
if self.parent.i2c_instances[bus_num] is None:
    self.parent.i2c_instances[bus_num] = machine.I2C(bus_num)
```

This lets components that share a physical bus (e.g. QwiicConnector and
Barometer both on I2C bus 2 on vB_1_1) reuse the same `machine.I2C` or
`pyb.SPI` object.

SPI bus allocation per board:

| Board | SPI Bus 1 | SPI Bus 2 | SPI Bus 3 |
|-------|-----------|-----------|-----------|
| vB_1_1 (F767) | Everest | -- | IMU (BNO085) |
| vB_2_0 (H747) | IMU (BNO085) | Everest | -- |
| vB_3_0 (H747) | IMU (BNO085), FTS | Everest | -- |

## Mode Switch (`mode_switch.py`)

The `ModeSwitch` class reads an ADC-based rotary (or dip) switch at boot to
determine which device type is connected. This drives the `config.py`
selection of the correct routing class and feature set.

### Pin Assignment

| Processor | Pin Reference | Notes |
|-----------|--------------|-------|
| STM32H747 (Portenta) | `C3_C` (cpu pin) | `machine.Pin.cpu.C3_C` |
| STM32F767 (Pyboard) | `W43` | Standard W-bus notation |

### Reading Sequence

1. Read the 16-bit unsigned ADC value (`read_u16()`).
2. Right-shift by 4 to produce a 12-bit value (`>> 4`).
3. Map the reading to a switch position using threshold ranges.
4. Map the switch position to a device ID via `id_from_position_dict`.
5. Map the device ID to a human-readable name via `name_from_id_dict`.

After power-up, `update()` should be called a second time after a short delay
to allow the switch voltage to settle.

### Position-to-ID Mapping

| Switch Position | ADC Range (12-bit) | Device ID | Device Name |
|----------------|--------------------|-----------|-------------|
| 0 | > 3800 | 0 | UNKNOWN |
| 1 | 2200 -- 2500 | 1 | HYBRID KNEE |
| 2 | 2500 -- 2900 | 1 | HYBRID KNEE |
| 3 | 1600 -- 1800 | 2 | POLYCENTRIC ANKLE |
| 4 | 2900 -- 3800 | 3 | POLYCENTRIC ANKLE ONLY |
| 5 | 1800 -- 1900 | 0 | UNKNOWN |
| 6 | 1900 -- 2200 | 0 | UNKNOWN |
| 7 | 1400 -- 1600 | 0 | UNKNOWN |
| 8 | 1100 -- 1400 | 8 | FOX KNEE |
| 9 | 800 -- 1100 | 0 | UNKNOWN |

Positions 1 and 2 both resolve to device ID 1 (HYBRID KNEE). Position 8
resolves to device ID 8 (FOX KNEE). Positions 0, 5, 6, 7, and 9 are currently
unmapped and resolve to UNKNOWN (device ID 0).

## Singleton Pattern

The `@singleton` decorator (defined in `decorators.py`) ensures that each
routing class (`UPythonLLCtrl_vB_1_1`, `UPythonLLCtrl_vB_2_0`,
`UPythonLLCtrl_vB_3_0`) has exactly one instance across the entire runtime.

```python
def singleton(cls, *args, **kw):
    instances = {}
    def _singleton():
        if cls not in instances:
            instances[cls] = cls(*args, **kw)
        return instances[cls]
    return _singleton
```

When `config.Board()` calls the routing class constructor, the decorator
intercepts the call and either creates a new instance (first call) or returns
the existing one. This guarantees that any reference to the component tree
(e.g. `components.Everest.SPI.bus_number`) points to the same object, and
modifications made by `config.py` (enabling/disabling components) are visible
everywhere.

## See Also

- [Device Configuration](../../documentation/device-configuration.md)
- [Module Index](./module-index.md)
- [Motor Control](./motor-control.md)
