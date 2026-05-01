import pyb                              # type: ignore
import array                            # type: ignore
import machine                          # type: ignore
from uctypes import addressof           # type: ignore

print('IMPORTING HARDWARE MANAGER')

class HardwareManager:
    """
    The HardwareManager class is responsible for initializing all hardware
    components of our control board. It acts as a support module tied to the
    main HybridLeg instance and creates instances of the hardware as
    attributes of the parent instance.
    """
    def __init__(self, parent):
        """
        Initialize hardware manager with reference to parent orchestrator
        module.

        Args:
            parent: Reference to the main HybridLeg instance
        """
        self.parent = parent

        # Define variables used to track I2C and SPI bus initialization so we
        # don't re-initialize by mistake. Note that we do one more than the
        # maximum bus number allowed for each below as the indexing is 1-based
        # for names (SPI 1-3) where arrays are zero-indexed (spi[0...]).
        self.parent.i2c_instances = [None] * (2 + 1)
        self.parent.spi_instances = [None] * (4 + 1)

    def initialize_all_hardware(self):
        """Main entry point to initialize all hardware components."""

        # The self.config file provides a class named 'Board()'. This class
        # creates an instance of the desired hardware routing class (i.e. class
        # named after hardware board revision which defines routing information).
        # By grabbing an instance of all components in the manner below, we now
        # can know which features are enabled (components.[feature].is_enabled()),
        # protocols used by features (components.[feature].protocol), and the
        # port/bus/pin number for interacting with a feature. This configuration
        # file should be defined within the application layer while the routing
        # class used resides within the 'routing.py' file for this package.
        # Please note that the 'Board()' class is designed such that any instance
        # you make to it shares one fixed memory region. Meaning, you can create
        # as many versions of 'components' as you like and changing one instance
        # will also change values on the other ones automatically. This allows
        # us to ensure one global 'Board()' and associated settings (flags)
        self.parent.components = self.parent.config.Board().get_components()
        
        print("~" * 120)
        print(
            "Processing %s v%s.%s.%s board components as defined in the "
            "configuration and routing files:"
            % (
                self.parent.components.board_name,
                self.parent.components.board_revision,
                self.parent.components.board_major,
                self.parent.components.board_minor
            )
        )

        # List of all setup methods
        setup_methods = [
            self.set_timers,
            # self.set_auxiliary_analog_in_variables,
            self.set_auxiliary_comm_variables,
            self.set_barometer_variables,
            self.set_braking_circuit_variables,
            self.set_buzzer_variables,
            self.set_everest_variables,
            self.set_external_comm_variables,
            self.set_external_led_variables,
            self.set_second_spi,
            self.set_imu_variables,
            self.set_fts_variables,
            self.set_joint_encoder_variables,
            self.set_qwiic_connector_variables,
            self.set_timing_output_variables,
        ]

        for method in setup_methods:
            method()

    # ========================================================================
    # Defining all timers
    # ========================================================================

    def set_timers(self):
        self.parent.interrupt_priorities = 5

        # ====================================================================
        #   Define all timers used by the device
        #
        #   The timers are defined in the 'device_parameters' class located in
        #   '../config.py'.
        #
        #   Channel assignments for PWM timers are set in the respective
        #   component initialization functions below and are also defined in
        #   the 'device_parameters' class.
        # --------------------------------------------------------------------

        # Primary Controller-Specific Timers
        if self.parent.device_parameters.PRIMARY_CONTROLLER:
            # IMU timer
            self.parent.timer_imu = pyb.Timer(
                self.parent.device_parameters.TIMER_IMU['ID'],
                freq=self.parent.device_parameters.TIMER_IMU['FREQ']
            )

            # State machine timer
            self.parent.timer_state_machine = pyb.Timer(
                self.parent.device_parameters.TIMER_STATE_MACHINE['ID'],
                freq=self.parent.device_parameters.TIMER_STATE_MACHINE['FREQ']
            )

        # Braking PWM timer
        if self.parent.components.BrakingCircuit.is_enabled():
            self.parent.timer_braking_pwm = pyb.Timer(
                self.parent.device_parameters.TIMER_BRAKING_PWM['ID'],
                freq=self.parent.device_parameters.TIMER_BRAKING_PWM['FREQ']
            )

        # CAPS send timer
        self.parent.timer_caps_send = pyb.Timer(
            self.parent.device_parameters.TIMER_CAPS_SEND['ID'],
            freq=self.parent.device_parameters.TIMER_CAPS_SEND['FREQ']
        )

        # Buzzer PWM timer
        self.parent.timer_buzzer_pwm = pyb.Timer(
            self.parent.device_parameters.TIMER_BUZZER_PWM['ID'],
            freq=self.parent.device_parameters.TIMER_BUZZER_PWM['FREQ']
        )

        # CAN bus external timer
        self.parent.timer_can_external = pyb.Timer(
            self.parent.device_parameters.TIMER_CAN_EXTERNAL['ID'],
            freq=self.parent.device_parameters.TIMER_CAN_EXTERNAL['FREQ']
        )

        # Main loop timer
        self.parent.timer_main_loop = pyb.Timer(
            self.parent.device_parameters.TIMER_MAIN_LOOP['ID'],
            freq=self.parent.device_parameters.TIMER_MAIN_LOOP['FREQ']
        )

        # Motor control timer
        self.parent.timer_motor_control = pyb.Timer(
            self.parent.device_parameters.TIMER_MOTOR_CONTROL['ID'],
            freq=self.parent.device_parameters.TIMER_MOTOR_CONTROL['FREQ']
        )

        # Assign interrupt priorities to timers
        for register in self.parent.device_parameters.NVIC_REGISTERS.values():
            self.parent.nvic.nvic_set_prio(
                register, self.parent.interrupt_priorities
            )

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Define AuxiliaryAnalogIn related variables
    def set_auxiliary_analog_in_variables(self):
        if self.parent.components.AuxiliaryAnalogIn.is_enabled():
            raise Exception(
                "No implementation available yet for 'AuxiliaryAnalogIn'. "
                "Aborting initialization!"
            )
        else:
            print("  ... Skipping 'AuxiliaryAnalogIn'")

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Define AuxiliaryComm related variables
    def set_auxiliary_comm_variables(self):
        try:
            if self.parent.components.AuxiliaryComm.is_enabled():
                raise Exception(
                    "No implementation available yet for 'AuxiliaryComm'. "
                    "Aborting initialization!"
                )
            else:
                print("  ... Skipping 'AuxiliaryComm'")
        except:
            pass

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Define Barometer related variables
    def set_barometer_variables(self):
        try:
            if self.parent.components.Barometer.is_enabled():

                # bar_flag = False
                # Value for initial barometer reading
                self.parent.bar_init = 0

                if (self.parent.components.Barometer.protocol
                        == self.parent.routing.Protocol.I2C):
                    print(
                        "  ... Preparing 'Barometer' using I2C %d"
                        % self.parent.components.Barometer.I2C.bus_number
                    )

                    # Ensure I2C bus is initialized
                    bus_num = self.parent.components.Barometer.I2C.bus_number
                    if self.parent.i2c_instances[bus_num] is None:
                        self.parent.i2c_instances[bus_num] = machine.I2C(
                            bus_num
                        )

                    # Define joint encoder
                    self.parent.barometer = (
                        self.parent.LPS25HB.LPS25HB(
                            self.parent.i2c_instances[bus_num]
                        )
                    )

                elif (self.parent.components.Barometer.protocol
                      == self.parent.routing.Protocol.SPI):
                    raise Exception(
                        "No SPI implementation available yet for 'Barometer' "
                        "(please use I2C). Aborting initialization!"
                    )

                else:
                    raise Exception(
                        "[CONFIG_ERROR]> Requested protocol '%d' for "
                        "'Barometer' is unsupported."
                        % self.parent.components.Barometer.protocol
                    )

            else:
                print("  ... Skipping 'Barometer'")
        except:
            pass

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Define BrakingCircuit related variables
    def set_braking_circuit_variables(self):
        if self.parent.components.BrakingCircuit.is_enabled():
            print("  ... Preparing 'BrakingCircuit'")

            braking_circuit_pwm_pin = pyb.Pin(
                self.parent.components.BrakingCircuit.PWM.pin
            )

            # Invert PWM signal if the processor is a Pyboard (Hybrid knee)
            # otherwise use normal PWM for the Portenta (Fox knee)
            if self.parent.components.BrakingCircuit.PWM.invert:
                pwm_pin_configuration = pyb.Timer.PWM_INVERTED
            else:
                pwm_pin_configuration = pyb.Timer.PWM

            self.parent.braking_circuit_pwm_ch = (
                self.parent.timer_braking_pwm.channel(
                    self.parent.device_parameters.TIMER_BRAKING_PWM['CHANNEL'],
                    mode=pwm_pin_configuration,
                    pin=braking_circuit_pwm_pin
                )
            )
            self.parent.braking_circuit_fet_enable = pyb.Pin(
                self.parent.components.BrakingCircuit.enable_fets,
                mode=pyb.Pin.OUT_PP,
                pull=pyb.Pin.PULL_NONE
            )

            # Ensure PWM is 0, or no braking
            self.parent.braking_circuit_pwm_ch.pulse_width_percent(0)

        else:
            print("  ... Skipping 'BrakingCircuit'")

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Define Buzzer related variables
    def set_buzzer_variables(self):
        if self.parent.components.Buzzer.is_enabled():
            print("  ... Preparing 'Buzzer'")

            buzzer_pwm_pin = pyb.Pin(
                self.parent.components.Buzzer.line_out,
                mode=pyb.Pin.OUT,
                pull=pyb.Pin.PULL_DOWN
            )

            self.parent.buzzer_ch = self.parent.timer_buzzer_pwm.channel(
                self.parent.device_parameters.TIMER_BUZZER_PWM['CHANNEL'],
                pyb.Timer.PWM,
                pin=buzzer_pwm_pin,
                pulse_width=0
            )

            self.parent.buzzer_ch.pulse_width_percent(0)

        else:
            print("  ... Skipping 'Buzzer'")

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Define Everest related variables
    def set_everest_variables(self):
        if self.parent.components.Everest.is_enabled():
            print("  ... Preparing 'Everest'")
            print(
                "Torque constant for large motor:     ",
                self.parent.CONST_DICT['TORQUE_CONSTANT_LARGE_MOTOR']
            )  # Nm/A

            self.parent.everest_status_word = 0
            self.parent.motor_velocity = self.parent.upy_float.upyFloat(0.0)

            #
            #   Initialize Everest SPI
            #

            # SPI speed depends on processor being used
            if self.parent.device_parameters.PROCESSOR == 'STM32H747':
                # Portenta
                baudrate = 50000000     # 50MHz
            elif self.parent.device_parameters.PROCESSOR == 'STM32F767':
                # Pyboard SF6
                baudrate = 30000000     # 30MHz (scales to 27MHz)

            bus_num = self.parent.components.Everest.SPI.bus_number
            self.parent.spi_instances[bus_num] = (
                self.parent.upy_spi.upySPI(bus_num)
            )
            self.parent.spi_instances[bus_num].init(
                mode=pyb.SPI.CONTROLLER,
                baudrate=baudrate,
                polarity=0,
                phase=1,
                bits=8,
                #   crc=0x1021,
                firstbit=pyb.SPI.MSB
            )

            print('Everest SPI:')
            print(self.parent.spi_instances[bus_num])

            #
            #   INITIALIZE EVEREST
            #

            # STO_1 line
            self.parent.everest_enable = pyb.Pin(
                self.parent.components.Everest.sto_1,
                mode=pyb.Pin.OUT_PP,
                pull=pyb.Pin.PULL_NONE
            )
            # STO_2 is controlled by DBB
            self.parent.everest_sto_2 = pyb.Pin(
                self.parent.components.Everest.sto_2,
                mode=pyb.Pin.IN
            )

            if (
                self.parent.device_parameters.device
                == self.parent.config.HYBRID_KNEE
                or self.parent.device_parameters.device
                == self.parent.config.FOX_KNEE
            ):
                self.parent.braking_circuit_fet_enable(0)
            # Enable Everest, while disabling braking
            self.parent.everest_enable(1)

            self.parent.servo = self.parent.novanta.Everest(
                self.parent.spi_instances[bus_num],
                self.parent.components.Everest.reset,
                self.parent.components.Everest.sync_1,
                self.parent.components.Everest.SPI.chip_select,
                self.parent.components.Everest.irq,
                config_dict=self.parent.CONST_DICT['EVEREST_CONFIG_DICT'],
                cyclic_pico_registers=(
                    self.parent.CONST_DICT['EVEREST_CYCLIC_PICO_REGISTERS']
                ),
                cyclic_poci_registers=(
                    self.parent.CONST_DICT['EVEREST_CYCLIC_POCI_REGISTERS']
                ),
                use_hardware_crc=True,
                verbose=False,
            )

            self.parent.nvic.nvic_set_prio(
                self.parent.servo.interrupt_pin.line() + 6,
                self.parent.interrupt_priorities - 1
            )
            # Set the initial set point for the current to 0.0
            self.parent.servo.channel_pico.current_quadrature_set_point = 0.0

            # Enable Everest
            self.parent.servo.state_transition(2)

            # Put Everest in cyclic mode
            self.parent.servo.enable_cyclic_mode()

            # Instantiate Everest controller
            self.parent.everest_pb = self.parent.controller.Controller(
                0,
                0,
                0,
                self.parent.CONST_DICT['TORQUE_CONSTANT_LARGE_MOTOR'],
                0
            )  # main controller for active mode impedance control

        else:
            print("  ... Skipping 'Everest'")

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Define ExternalComm (e.g. CAN) related variables
    def set_external_comm_variables(self):
        if self.parent.components.ExternalComm.is_enabled():

            if (self.parent.components.ExternalComm.protocol
                    == self.parent.routing.Protocol.CAN):
                print(
                    "  ... Preparing 'ExternalComm' using CAN %d"
                    % self.parent.components.ExternalComm.CAN.bus_number
                )
                self.parent.can = pyb.CAN(
                    self.parent.components.ExternalComm.CAN.bus_number
                )
                if self.parent.device_parameters.PROCESSOR == 'STM32H747':
                    self.parent.can.init(
                        pyb.CAN.NORMAL,
                        sample_point=50,
                        baudrate=1_000_000,
                        brs_baudrate=1_000_000,
                        auto_restart=True
                    )
                elif self.parent.device_parameters.PROCESSOR == 'STM32F767':
                    self.parent.can.init(
                        pyb.CAN.NORMAL,
                        prescaler=6,
                        sjw=1,
                        bs1=5,
                        bs2=3,
                        auto_restart=True
                    )
                self.parent.can_send = (
                    self.parent.can_sender.canSender(self.parent.can)
                )
            else:
                raise Exception(
                    "[CONFIG_ERROR]> Requested protocol '%d' for "
                    "'ExternalComm' is unsupported."
                    % self.parent.components.ExternalComm.protocol
                )

        else:
            print("  ... Skipping 'ExternalComm'")

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Define ExternalLEDs related variables
    def set_external_led_variables(self):
        if self.parent.components.ExternalLEDs.is_enabled():
            print(
                "  ... Preparing 'ExternalLEDs' as individual RGB LEDs "
                "(generic GPIO on/off)"
            )
            self.parent.external_led_blue = pyb.Pin(
                self.parent.components.ExternalLEDs.blue,
                mode=pyb.Pin.OUT,
                pull=pyb.Pin.PULL_DOWN
            )
            self.parent.external_led_green = pyb.Pin(
                self.parent.components.ExternalLEDs.green,
                mode=pyb.Pin.OUT,
                pull=pyb.Pin.PULL_DOWN
            )
            self.parent.external_led_red = pyb.Pin(
                self.parent.components.ExternalLEDs.red,
                mode=pyb.Pin.OUT,
                pull=pyb.Pin.PULL_DOWN
            )
        else:
            print("  ... Skipping 'ExternalLEDs'")

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Setup second SPI bus for IMU and FTS

    def set_second_spi(self):
        if self.parent.components.IMU.is_enabled() or self.parent.components.ForceTorqueSensor.is_enabled():
            print("  ... Preparing SPI for IMU and/or FTS")
            #SPI speed: 4 MHz for both processors. On H747 this prescales
            # to ~3.125 MHz, compatible with BNO085 (max 3 MHz) and ADS1292
            # (max ~4.25 MHz) which share SPI bus 1 on vB_3_0.
            if self.parent.device_parameters.PROCESSOR == 'STM32H747':
                # Portenta
                baudrate = 4000000      # 4MHz (scales to ~3.125MHz)
            elif self.parent.device_parameters.PROCESSOR == 'STM32F767':
                # Pyboard SF6
                baudrate = 4000000      # 4MHz (scales to 3.375MHz)

            if self.parent.components.IMU.is_enabled():
                bus_num = self.parent.components.IMU.SPI.bus_number
                self.parent.spi_instances[bus_num] = (
                        self.parent.upy_spi.upySPI(bus_num)
                )
            else:
                bus_num = self.parent.components.ForceTorqueSensor.SPI.bus_number
                self.parent.spi_instances[bus_num] = (
                        self.parent.upy_spi.upySPI(bus_num)
                )
            
            self.parent.spi_instances[bus_num].init(
                mode=pyb.SPI.MASTER,
                baudrate=baudrate,
                polarity=1,
                phase=1,
                bits=8,
                firstbit=pyb.SPI.MSB
            )
            
            print("SPI", self.parent.spi_instances[bus_num])

        else:
            print("  ... Skipping SPI setup for IMU and FTS")

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Define IMU related variables
    def set_imu_variables(self):
        if self.parent.components.IMU.is_enabled():
            print("  ... Preparing 'IMU'")
            # IMU arrays
            # [thigh, shank, ax, ay, az, gyrox, gyroy, gyroz]
            self.parent.imu_data = array.array(
                'f',
                [0, 0, 0, 0, 0, 0, 0, 0]
            )
            self.parent.imu_qpoint = array.array('f', [2**8, 2**9])

            # Values required to calculate the arctan2 to convert the
            # IMU values into thigh and shank angles.
            self.parent.arc_tan2_taylor_coefs = array.array(
                'f',
                [
                    0,                  # ┌        ┐
                    0.3333333,          # │ TAYLOR │
                    0.2,                # │ SERIES │
                    0.142857,           # │ PARAMS │
                    0.111111,           # └        ┘
                    57.2958,            # 180 / pi
                    (1 / (1 << 14)),    # formatting value
                    2.0,                # Used in calulating preliminary calc
                    0.0,                # save output of preliminary calc.
                    1.0,                # Used in calulating preliminary calc
                    1.570796,           # pi / 2
                    0,                  # numerator of val
                    0,                  # denominator of val
                    1,                  # multiplier for when knee inverted
                ]
            )

            print('BNO SPI:')
            bus_num = self.parent.components.IMU.SPI.bus_number
            print(self.parent.spi_instances[bus_num])

            self.parent.imu = self.parent.BNO085.BNO085(
                self.parent.spi_instances[bus_num],                     # spi_object
                self.parent.components.IMU.SPI.chip_select,             # h_csn_pin_id - chip select
                self.parent.components.IMU.protocol_select_pin_0,       # ps0_wake_pin_id
                self.parent.components.IMU.protocol_select_pin_1,       # ps1_pin_id
                self.parent.components.IMU.bootloader_mode_select,      # bootn_pin_id
                self.parent.components.IMU.reset,                       # nrst_pin_id
                self.parent.components.IMU.data_is_ready,               # intn_pin_id
                verbose=False,                
                report_type=[
                    0x05,  # BNO085._REPORT_ROTATION_VECTOR,
                    0x04,  # BNO085._REPORT_LINEAR_ACCELERATION,
                    0x02,  # BNO085._REPORT_GYROSCOPE_CALIBRATED,
                    # BNO085._REPORT_PRESSURE (will default to 44000 us min)
                    0x0A
                ],
                # microseconds (5000 us is 200Hz) (20000 us is 50Hz)
                report_interval=20000,
                interrupt_priority=self.parent.interrupt_priorities)

            self.parent.scale_imu_for_sm = array.array('f', [36, 409.6])
            self.parent.imu_isr_addresses = array.array(
                'i', [
                    addressof(self.parent.arc_tan2_taylor_coefs),       # 0
                    addressof(self.parent.imu.raw_quaternion_array),    # 4
                    addressof(self.parent.imu.raw_gyroscope_array),     # 8
                    addressof(self.parent.imu.raw_acceleration_array),  # 12
                    addressof(self.parent.imu_qpoint),                  # 16
                    addressof(self.parent.imu_data),                    # 20
                    addressof(self.parent.PARAM_DICT['JOINT_ANGLE']),   # 24
                    addressof(self.parent.sm_channels_current),         # 28
                    addressof(self.parent.scale_imu_for_sm)             # 32
                ]
            )

            raw_pressure = self.parent.imu.raw_pressure_array
            self.parent.PARAM_DICT['BAR_BYTE_VALS'][0] = (
                self.parent.imu.q_to_float_32(
                    (raw_pressure[1] << 16) + raw_pressure[0], 20
                )
            )

        else:
            print("  ... Skipping 'IMU'")
            # this should be defined under bno085 in the master self.config
            _BNO_NRST_PIN = 'J2_16'
            # active low
            self.parent.nrst_pin = pyb.Pin(
                _BNO_NRST_PIN,
                pyb.Pin.OUT_OD,
                pull=pyb.Pin.PULL_UP
            )
            self.parent.nrst_pin.value(False)

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Define FTS related variables
    def set_fts_variables(self):
        try:
            if self.parent.components.ForceTorqueSensor.is_enabled():
                fts_unit = self.parent.tunable_settings.FTS_UNIT
                if fts_unit is None:
                    print("  ... Skipping 'ForceTorqueSensor' (FTS_UNIT not set)")
                    return

                from . import fts_parameters
                params = fts_parameters.get_parameters(fts_unit)
                if params is None:
                    print("  ... ERROR: No parameters for FTS unit %d" % fts_unit)
                    return

                print("  ... Preparing 'ForceTorqueSensor' (unit %d)" % fts_unit)

                # Disable BNO085 interrupt to prevent SPI bus collisions
                # during ADS1292 configuration. The BNO085 ISR does full
                # SPI transactions on the shared bus, which would corrupt
                # ADS1292 register writes if it fires mid-configuration.
                imu_intn_disabled = False
                if hasattr(self.parent, 'imu'):
                    self.parent.imu.intn_pin.disable()
                    imu_intn_disabled = True


                # Enable peripheral power for external SPI devices (active low)
                self.parent.peripheral_power_pin = pyb.Pin(
                    self.parent.components.PeripheralPower.external_spi_power,
                    mode=pyb.Pin.OUT_OD,
                    pull=pyb.Pin.PULL_UP
                )
                self.parent.peripheral_power_pin.value(0)
                pyb.delay(100)

                # Reuse SPI bus 1 (shared with IMU, already initialized)
                bus_num = self.parent.components.ForceTorqueSensor.SPI.bus_number
                spi = self.parent.spi_instances[bus_num]
                print('bus num', bus_num)
                print(spi)
                
                # """Initialize Force/Torque Sensor."""
                print("Initializing Force/Torque Sensor...")
                self.parent.fts_sensor = self.parent.fts.FTS(
                    spi_object=spi,
                    cs_pin_id=self.parent.components.ForceTorqueSensor.SPI.chip_select,
                    drdy_pin_id=self.parent.components.ForceTorqueSensor.data_ready,
                    reset_pin_id=self.parent.components.ForceTorqueSensor.power_reset,
                    cal_fz=params['cal_fz'],
                    cal_my=params['cal_my'],
                    verbose=True,
                    interrupt_priority=self.parent.interrupt_priorities,
                    transient_reject_threshold=self.parent.CONST_DICT['FTS_TRANSIENT_REJECT_THRESHOLD']
                )

                # Apply V0 from parameters
                self.parent.fts_sensor.v0[0] = params['v0'][0]
                self.parent.fts_sensor.v0[1] = params['v0'][1]

                # Store NIDs for CAPS output handler setup
                self.parent.fts_nid_control = params['nid_control']
                self.parent.fts_nid_data1 = params['nid_data1']
                self.parent.fts_nid_data2 = params['nid_data2']

                self.parent.fts_sensor.start_conversions()

                # Re-enable BNO085 interrupt now that ADS1292 is configured
                # and streaming. Both ExtInts are at the same NVIC priority
                # so neither can preempt the other mid-SPI-transaction.
                if imu_intn_disabled:
                    self.parent.imu.intn_pin.enable()

            else:
                print("  ... Skipping 'ForceTorqueSensor'")
        except AttributeError:
            print("  ... Skipping 'ForceTorqueSensor' (not defined on this board)")

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Define JointEncoder related variables
    def set_joint_encoder_variables(self):
        try:
            if self.parent.components.JointEncoder.is_enabled():

                # Support for legacy devices using an external ADS IC over I2C
                # for reading in joint encoder values
                if (self.parent.components.JointEncoder.protocol
                        == self.parent.routing.Protocol.I2C):
                    print(
                        "  ... Preparing 'JointEncoder' using I2C %d"
                        % self.parent.components.JointEncoder.I2C.bus_number
                    )

                    # Ensure I2C bus is initialized
                    bus_num = self.parent.components.JointEncoder.I2C.bus_number
                    if self.parent.i2c_instances[bus_num] is None:
                        self.parent.i2c_instances[bus_num] = pyb.I2C(bus_num)

                # Current devices utilize a direct analog read
                elif (self.parent.components.JointEncoder.protocol
                      == self.parent.routing.Protocol.ANALOG):
                    print(
                        "  ... Preparing 'JointEncoder' using ADC on pin %s"
                        % self.parent.components.JointEncoder.channel_index
                    )
                    self.parent.joint_encoder = pyb.ADC(
                        self.parent.components.JointEncoder.channel_index
                    )

                else:
                    raise Exception(
                        "[CONFIG_ERROR]> The 'JointEncoder' component is "
                        "enabled but the requested protocol '%s' is unsupported."
                        % self.parent.components.JointEncoder.protocol
                    )

            else:
                print("  ... Skipping 'JointEncoder'")
        except:
            pass

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Define QwiicConnector related variables
    def set_qwiic_connector_variables(self):
        if self.parent.components.QwiicConnector.is_enabled():

            # Support for legacy devices using an external ADS IC over I2C
            # for reading in Qwiic connector values
            if (self.parent.components.QwiicConnector.protocol
                    == self.parent.routing.Protocol.I2C):
                print(
                    "  ... Preparing 'QwiicConnector' using I2C %d"
                    % self.parent.components.QwiicConnector.I2C.bus_number
                )

                # Ensure I2C bus is initialized
                bus_num = self.parent.components.QwiicConnector.I2C.bus_number
                if self.parent.i2c_instances[bus_num] is None:
                    print('initializing I2C bus', bus_num)
                    self.parent.i2c_instances[bus_num] = pyb.I2C(
                        bus_num,
                        pyb.I2C.CONTROLLER
                    )
                    print(
                        'initialized I2C bus',
                        self.parent.i2c_instances[bus_num]
                    )
                pyb.delay(1)  # Wait 1ms after powering VL6108X

            else:
                raise Exception(
                    "[CONFIG_ERROR]> The 'QwiicConnector' component is "
                    "enabled but the requested protocol '%s' is unsupported."
                    % self.parent.components.JointEncoder.protocol
                )

        else:
            print("  ... Skipping 'QwiicConnector'")

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Define TimingOutput related variables
    def set_timing_output_variables(self):
        try:
            if self.parent.components.TimingOutput.is_enabled():
                print("  ... Preparing 'TimingOutput'")
                self.parent.timing_pin = pyb.Pin(
                    self.parent.components.TimingOutput.line_out,
                    mode=pyb.Pin.OUT_PP,
                    pull=pyb.Pin.PULL_UP,
                    value=1
                )
            else:
                print("  ... Skipping 'TimingOutput'")
        except:
            pass
        