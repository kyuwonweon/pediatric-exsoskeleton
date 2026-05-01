####################################################################################################
# @file
#
# @brief            Defines board-specific routing details.
#
# @WARNING          THIS FILE SHOULD NOT BE MODIFIED UNLESS THE HARDWARE OR WIRING HARNESS HAS BEEN
#                   MODIFIED. IF YOU WISH TO TWEAK FEATURES FOR YOUR APPLICATION, PLEASE DO SO IN
#                   YOUR 'config.py' FILE IN YOUR APPLICATION LAYER.
####################################################################################################


# ==================================================================================================
# Load in external dependencies
# --------------------------------------------------------------------------------------------------
from . import decorators

print('IMPORTING ROUTING')


####################################################################################################
# @class            Protocol
#
# @brief            Defines all supported protocols.
#
# @note             Intended to be used as rvalue for Component{}.protocol.
####################################################################################################
class Protocol(object):
    # Note:  '0' reserved for 'not set'
    ANALOG          = 1                 # Component is an analog device (input or output).
    CAN             = 2                 # Component communicates over CAN.
    GPIO            = 3                 # Component utilizes (a) generic GPIO pin(s).
    I2C             = 4                 # Component communicates over I2C.
    RS_232          = 5                 # Component communicates over RS-232.
    RS_485          = 6                 # Component communicates over RS-485.
    SK6812          = 7                 # Component is a LED of type SK6812 and requires bit banging the communication protocol.
    SPI             = 8                 # Component communicates over SPI.
    UART            = 9                 # Component communicates over UART.


####################################################################################################
# @class            Component
#
# @brief            Base class for defining a component. A component is first-level entry within a
#                   routing class.
####################################################################################################
class Component(object):

    ################################################################################################
    # @brief        Main initialization function for class
    #
    # @param[in]    self                Object pointer to class instance.
    ################################################################################################
    def __init__(self):
        self._protocol  = None
        self._active    = False

    ################################################################################################
    # @brief        Returns the present value for 'instance.protocol'.
    #
    # @note         Uses a Python "@property decorator" to achieve the setter for the field.
    #
    # @param[in]    self                Object pointer to class instance.
    #
    # @returns      Present value of 'self._protocol'.
    ################################################################################################
    @property
    def protocol(self):
        return self._protocol

    ################################################################################################
    # @brief        Updates the value for the 'instance.protocol' with the value provided.
    #
    # @note         Uses a Python "@[parameter-name].getter decorator" to achieve the getter for the
    #               field.
    #
    # @param[in]    self                Object pointer to class instance.
    # @param[in]    value               Protocol value to assign. Should be member of 'Protocol{}'.
    ################################################################################################
    @protocol.setter
    def protocol(self, value):
        self._protocol = value

    ################################################################################################
    # @brief        Disables the comopnent.
    #
    # @details      Changes the internal 'self._active' flag to 'False'.
    #
    # @param[in]    self                Object pointer to class instance.
    ################################################################################################
    def disable(self):
        self._active = False

    ################################################################################################
    # @brief        Enables the comopnent.
    #
    # @details      Changes the internal 'self._active' flag to 'True'.
    #
    # @param[in]    self                Object pointer to class instance.
    ################################################################################################
    def enable(self):
        self._active = True

    ################################################################################################
    # @brief        Returns a boolean flag indicating if the component enabled.
    #
    # @param[in]    self                Object pointer to class instance.
    #
    # @returns      Boolean flag indicating is component should be enabled (True) or disabled
    #               (False).
    ################################################################################################
    def is_enabled(self):
        return self._active


####################################################################################################
# @class            UPythonLLCtrl_vB_1_1
#
# @brief            Aggregates all hardware specific routing and wiring information for the second
#                   iteration of the lower limb controller PCB (upython-ll-ctrl vB1.1).
#
# @details          All routing references use the X and Y pins when available and W pin notation
#                   when W-Bus only access point.
#
# @warning          The decorator used ensures only a single instance of this class is used across
#                   all Python code (singleton implementation).
####################################################################################################
@decorators.singleton
class UPythonLLCtrl_vB_1_1(object):

    # ==============================================================================================
    # Define identifying information for the platform this class targets. Breaks down the board
    # name and version information into the variables below.
    # ----------------------------------------------------------------------------------------------
    board_name                              = "upython-ll-ctrl"  # Official name of PCB
    board_revision                          = "B"                # Revision for PCB (A, B, ...)
    board_major                             = 1                  # Major number for PCB
    board_minor                             = 1                  # Minor number for PCB

    ################################################################################################
    # @brief        Initialization function for class.
    #
    # @details      Instantiates all nested/inner classes and assigns a default protocol to each
    #               component. By default, all components will be disabled and callers can both
    #               override the default protocols and enable the components they wish to utilize.
    #
    # @param[in]    self                Object pointer to class instance.
    ################################################################################################
    def __init__(self):
        # Create an instance of all first-level nested/inner classes within this class. Doing so in
        # the following manner allows us to make each inner class of type 'Component' and have
        # external callers modify the state/value of each 'Component' (without instantiation we
        # cannot effectively enable, disable, set protocol values, etc.).
        self.AuxiliaryAnalogIn              = self._AuxiliaryAnalogIn()
        # self.AuxiliaryComm                  = self._AuxiliaryComm()
        self.BrakingCircuit                 = self._BrakingCircuit()
        self.Buzzer                         = self._Buzzer()
        self.Everest                        = self._Everest()
        self.ESCON                          = self._ESCON()
        self.ExternalComm                   = self._ExternalComm()
        self.ExternalLEDs                   = self._ExternalLEDs()
        self.IMU                            = self._IMU()
        self.InternalLEDs                   = self._InternalLEDs()
        self.ModeSelectionSwitch            = self._ModeSelectionSwitch()
        self.QwiicConnector                 = self._QwiicConnector()
        self.SafetySwitch                   = self._SafetySwitch()
        self.TimingOutput                  = self._TimingOutput()

        # Select the desired protocol to use for any and all component objects within this class.
        self.AuxiliaryAnalogIn.protocol     = Protocol.ANALOG
        # self.AuxiliaryComm.protocol         = Protocol.UART
        self.BrakingCircuit.protocol        = Protocol.GPIO
        self.Buzzer.protocol                = Protocol.GPIO
        self.Everest.protocol               = Protocol.SPI
        self.ESCON.protocol                 = Protocol.GPIO
        self.ExternalComm.protocol          = Protocol.CAN
        self.ExternalLEDs.protocol          = Protocol.GPIO
        self.IMU.protocol                   = Protocol.SPI
        self.InternalLEDs.protocol          = Protocol.GPIO
        self.ModeSelectionSwitch.protocol   = Protocol.ANALOG
        self.QwiicConnector.protocol        = Protocol.I2C
        self.SafetySwitch.protocol          = Protocol.ANALOG

    ################################################################################################
    # @class        _AuxiliaryAnalogIn
    #
    # @brief        Defines pin routing information for interacting with auxiliary analog lines.
    #
    # @note         The auxiliary analog lines listed are ADC lines routed to external headers that
    #               are not already in used (e.g. JointEncoder.channel_index uses channel_a_index so
    #               that channel is not listed below).
    #
    # @warning      Two of the channels below (C and D) overlap with the AuxilaryCOM{} entries for
    #               Rx and Tx. Make sure to be selective in how you utilize these two lines.
    ################################################################################################
    class _AuxiliaryAnalogIn(Component):
        channel_a_index         = 'W19'                     # Index for analog channel-a data.
        channel_b_index         = 'W17'                     # Index for analog channel-b data.
        channel_c_index         = 'W15'                     # Index for analog channel-c data. WARNING: same pin as 'AuxiliaryComm{}.UART{}.tx'.
        channel_d_index         = 'W11'                     # Index for analog channel-d data. WARNING: same pin as 'AuxiliaryComm{}.UART{}.rx'.

    ################################################################################################
    # @class        _BrakingCircuit
    #
    # @brief        Defines pin routing information for the "dynamic braking circuit".
    ################################################################################################
    class _BrakingCircuit(Component):
        enable_fets             = 'W54'                     # Digital output to enable braking mode (value > 0) or disable braking mode (value = 0).

        ############################################################################################
        # @class    PWM
        #
        # @brief    Defines pin routing information for the PWM output pin driving the braking
        #           circuit to control the rate of braking on the main motor (faster = more braking
        #           and stiffer joint).
        #
        # @note     Only relevant if the 'enable_fets' line is active.
        ############################################################################################
        class PWM(object):
            pin                 = 'W52'                     # Line to drive for PWM output.
            channel             = 3                         # Channel number for time to use for PWM output.
            invert              = True                      # Invert the PWM signal.

    ############################################################################################
    # @class    _Buzzer
    #
    # @brief    Defines pin routing information for driving the external buzzer via a DAC pin.
    ############################################################################################
    class _Buzzer(Component):
        line_out                = 'W19'                     # Line to drive when controlling the external buzzer

    ################################################################################################
    # @class        _Everest
    #
    # @brief        Defines pin routing information for interacting with the Novanta Everest motor
    #               controller.
    #
    # @note         There is no direct control over the two STO lines for this platform. Instead:
    #                   * STO1 is the opposite state of 'BrakingCircuit.enable_fets'.
    #                   * STO2 is connected to 'dynamic braking status'?
    ################################################################################################
    class _Everest(Component):
        sto_1                   = 'W49'                     # STO1 control line for the motor controller. Set high to enable.
        sto_2                   = 'W45'                     # STO2 is controlled by DBB, and connected as GPIO input to pyboard.

        sync_0                  = 'W17'
        sync_1                  = 'W11'                     # boot/sync_1 pin
        irq                     = 'W24'
        reset                   = 'W22'

        ############################################################################################
        # @class    SPI
        #
        # @brief    Defines pin routing information for the SPI bus of the Everest servo
        ############################################################################################
        class SPI(object):
            bus_number          = 1                         # Name/ID/Number of target SPI bus.
            chip_select         = 'W7'                      # SPI chip select line.

    ################################################################################################
    # @class        _ESCON
    #
    # @brief        Defines pin routing information for interacting with the ESCON motor controller.
    ################################################################################################
    class _ESCON(Component):
        enable_motor            = 'W73'                     # Digital output line to enable or disable the ESCON motor controller.
        measured_current        = 'W63'                     # Analog input line from ESCON with value represening the present current draw of the motor.
        motor_spin_direction    = 'W74'                     # Digital output line for direction to spin the motor.
        fault_status            = 'W71'                     # Digital input line from ESCON with value representing the error fault state of the escon.

        ############################################################################################
        # @class    MotorTorque
        #
        # @brief    Defines pin routing information for setting the motor speed for the ESCON using
        #           a DAC pin.
        ############################################################################################
        class MotorTorque(object):
            channel             = 1
            line_out            = 'W15'                     # Servo command (ESCON-SPD-SET-PWM on schematic)

    ################################################################################################
    # @class        _ExternalComm
    #
    # @brief        Defines pin routing information for the main [external] communication bus.
    ################################################################################################
    class _ExternalComm(Component):
        ############################################################################################
        # @class    CAN
        #
        # @brief    Defines pin routing information for the CAN bus used as main communication bus.
        ############################################################################################
        class CAN(object):
            bus_number              = 1                     # Name/ID/Number of target CAN bus.
            rx                      = 'X9'                  # CAN receive line used in CAN communication. Connected to output [RXD] line from CAN transceiver.
            tx                      = 'X10'                 # CAN transmit line used in CAN communication. Connected to input [TXD] line to CAN transceiver.

    ############################################################################################
    # @class        _ExternalLEDs
    #
    # @brief        Defines pin routing information for the external LEDs provided by CBM.
    #
    # @note         LED requires bit banging in assembly code to communicate. Make sure to set
    #               the protocol type to "Protocol.SK6812".
    ############################################################################################
    class _ExternalLEDs(Component):
        blue                    = 'W27'                     # GPIO line for controlling the blue LED.
        green                   = 'W67'                     # GPIO line for controlling the green LED.
        red                     = 'W65'                     # GPIO line for controlling the red LED.

    ################################################################################################
    # @class        _IMU
    #
    # @brief        Defines pin routing information for interacting with the external IMU chip.
    ################################################################################################
    class _IMU(Component):
        bootloader_mode_select  = 'W34'                     # Bootloader mode select pin.
        data_is_ready           = 'W33'                     # Input pin used to inform us that the IMU chip has data ready to be sampled (queried) over I2C.
        protocol_select_pin_0   = 'W23'                     # Protocol select pin 0 [also used to wake processor in SPI mode].
        protocol_select_pin_1   = 'W26'                     # Protocol select pin 1.
        reset                   = 'W25'                     # Reset pin (active low).

        ############################################################################################
        # @class    I2C
        #
        # @brief    Defines pin routing information for the I2C bus on the BNO085 IMU.
        ############################################################################################
        class I2C(object):
            address             = 0x4A                      # Address of target on I2C bus.
            bus_number          = 2                         # Name/ID/Number of target I2C bus.
            scl                 = 'Y9'                      # SCL pin for I2C communication.
            sda                 = 'Y10'                     # SDA pin for I2C communication.

        ############################################################################################
        # @class    SPI
        #
        # @brief    Defines pin routing information for the SPI bus on the BNO085 IMU.
        ############################################################################################
        class SPI(object):
            bus_number          = 3                         # Name/ID/Number of target SPI bus.
            chip_select         = 'W16'                     # SPI chip select line.
            clock               = 'W29'                     # SPI clock line.
            miso                = 'W50'                     # SPI MISO line.
            mosi                = 'W46'                     # SPI MOSI line.

    ############################################################################################
    # @class        _InternalLEDs
    #
    # @brief        Defines pin routing information for the on-board LEDs provided with the
    #               active MicroPython development board.
    ############################################################################################
    class _InternalLEDs(Component):
        blue                    = 3                         # ID number for controlling the blue LED.
        green                   = 2                         # ID number for controlling the green LED.
        red                     = 1                         # ID number for controlling the red LED.

    ################################################################################################
    # @class        _ModeSelectionSwitch
    #
    # @brief        Defines pin routing information for interacting with the rotary position switch.
    #
    # @details      Mode selection is done via an external 10 position switch. It provides an analog
    #               value (line_in) that breaks down as follows:
    #                   * Position-0:   3.300 V
    #                   * Position-1:   1.800 V
    #                   * Position-2:   2.031 V
    #                   * Position-3:   1.342 V
    #                   * Position-4:   2.383 V
    #                   * Position-5:   1.488 V
    #                   * Position-6:   1.642 V
    #                   * Position-7:   1.161 V
    #                   * Position-8:   0.943 V
    #                   * Position-9:   0.762 V
    ################################################################################################
    class _ModeSelectionSwitch(Component):
        line_in                 = 'W53'                     # Analog input line to same to read state of rotary switch.

    ################################################################################################
    # @class        _QwiicConnector
    #
    # @brief        Defines pin routing information for interacting with peripherals attached to the
    #               system via the external Qwiic connector.
    ################################################################################################
    class _QwiicConnector(Component):

        ############################################################################################
        # @class    I2C
        #
        # @brief    Defines pin routing information for the I2C bus connected to the Qwiic connector.
        #
        # @note     No address information is tracked here as the remote device address is not a
        #           fixed value.
        ############################################################################################
        class I2C(object):
            bus_number          = 2                         # Name/ID/Number of target I2C bus.
            scl                 = 'Y9'                      # SCL pin for I2C communication. W12
            sda                 = 'Y10'                     # SDA pin for I2C communication. W8

    ################################################################################################
    # @class        _SafetySwitch
    #
    # @brief        Defines pin routing information for interacting with the external safety switch.
    ################################################################################################
    class _SafetySwitch(Component):
        line_in                 = 'Y11'                     # Input pin for reading state of external safety switch.

    ################################################################################################
    # @class        _TimingOutput
    #
    # @brief        Defines pin routing information for interacting with a GPIO used to time
    #               sections of code
    #
    #               NOTE: pin used is allocated for EXT-SPI-IRQ on connector J4, which allows for
    #               additional SPI peripherals to be connected to SPI[3] in the future
    ################################################################################################
    class _TimingOutput(Component):
        line_out                = 'W47'          # W51


####################################################################################################
# @class            UPythonLLCtrl_vB_2_0
#
# @brief            Aggregates all hardware specific routing and wiring information for the proposed
#                   Portenta lower limb controller PCB.
#
# @details          All routing references use the X and Y pins when available and W pin notation
#                   when W-Bus only access point.
#
# @warning          The decorator used ensures only a single instance of this class is used across
#                   all Python code (singleton implementation).
####################################################################################################
@decorators.singleton
class UPythonLLCtrl_vB_2_0(object):

    # ==============================================================================================
    # Define identifying information for the platform this class targets. Breaks down the board
    # name and version information into the variables below.
    # ----------------------------------------------------------------------------------------------
    board_name                              = "upython-ll-ctrl"          # Official name of PCB
    board_revision                          = "B"                        # Revision for PCB (A, B, ...)
    board_major                             = 2                          # Major number for PCB
    board_minor                             = 0                          # Minor number for PCB

    ################################################################################################
    # @brief        Initialization function for class.
    #
    # @details      Instantiates all nested/inner classes and assigns a default protocol to each
    #               component. By default, all components will be disabled and callers can both
    #               override the default protocols and enable the components they wish to utilize.
    #
    # @param[in]    self                Object pointer to class instance.
    ################################################################################################
    def __init__(self):
        # Create an instance of all first-level nested/inner classes within this class. Doing so in
        # the following manner allows us to make each inner class of type 'Component' and have
        # external callers modify the state/value of each 'Component' (without instantiation we
        # cannot effectively enable, disable, set protocol values, etc.).
        self.BrakingCircuit                 = self._BrakingCircuit()
        self.Buzzer                         = self._Buzzer()
        self.ESCON                          = self._ESCON()
        self.Everest                        = self._Everest()
        self.ExternalComm                   = self._ExternalComm()
        self.ExternalLEDs                   = self._ExternalLEDs()
        # self.GPIOExpander                   = self._GPIOExpander()
        self.IMU                            = self._IMU()
        self.InternalLEDs                   = self._InternalLEDs()
        self.ModeSelectionSwitch            = self._ModeSelectionSwitch()
        self.QwiicConnector                 = self._QwiicConnector()
        # self.SPIExternal                    = self._SPIExternal()
        self.TimingOutput                   = self._TimingOutput()

        # Select the desired protocol to use for any and all component objects within this class.
        self.BrakingCircuit.protocol        = Protocol.GPIO
        self.Buzzer.protocol                = Protocol.GPIO
        self.ESCON.protocol                 = Protocol.GPIO
        self.Everest.protocol               = Protocol.SPI
        self.ExternalComm.protocol          = Protocol.CAN
        self.ExternalLEDs.protocol          = Protocol.GPIO
        # self.GPIOExpander.protocol          = Protocol.SPI
        self.IMU.protocol                   = Protocol.SPI
        self.InternalLEDs.protocol          = Protocol.GPIO
        self.ModeSelectionSwitch.protocol   = Protocol.ANALOG
        self.QwiicConnector.protocol        = Protocol.I2C
        # self.SPIExternal.protocol           = Protocol.SPI

    ################################################################################################
    # @class        _BrakingCircuit
    #
    # @brief        Defines pin routing information for the "dynamic braking circuit".
    ################################################################################################
    class _BrakingCircuit(Component):
        enable_fets             = 'J2_65'                   # Digital output to enable braking mode (value > 0) or disable braking mode (value = 0).

        ############################################################################################
        # @class    PWM
        #
        # @brief    Defines pin routing information for the PWM output pin driving the braking
        #           circuit to control the rate of braking on the main motor (faster = more braking
        #           and stiffer joint).
        #
        # @note     Only relevant if the 'enable_fets' line is active.
        ############################################################################################
        class PWM(object):
            pin                 = 'J2_60'                   # Line to drive for PWM output.
            invert              = False                     # Invert the PWM signal.

    ############################################################################################
    # @class    _Buzzer
    #
    # @brief    Defines pin routing information for driving the external buzzer via a DAC pin.
    ############################################################################################
    class _Buzzer(Component):
        line_out                = 'J2_61'                   # Line to drive when controlling the external buzzer

    ################################################################################################
    # @class        _ESCON
    #
    # @brief        Defines pin routing information for interacting with the ESCON motor controller.
    ################################################################################################
    class _ESCON(Component):
        enable_motor            = 'J2_6'                    # Digital output line to enable or disable the ESCON motor controller.
        measured_current        = 'J2_73'                   # Analog input line from ESCON with value represening the present current draw of the motor.
        motor_spin_direction    = 'J2_4'                    # Digital output line for direction to spin the motor.
        fault_status            = 'J2_8'                    # Digital input line from ESCON with value representing the error fault state of the escon.

        ############################################################################################
        # @class    MotorTorque
        #
        # @brief    Defines pin routing information for setting the motor speed for the ESCON using
        #           a DAC pin.
        ############################################################################################
        class MotorTorque(object):
            line_out            = 'J2_59'                   # Servo command (ESCON-SPD-SET-PWM on schematic)

    ################################################################################################
    # @class        _Everest
    #
    # @brief        Defines pin routing information for interacting with the Novanta Everest motor
    #               controller.
    #
    # @note         There is no direct control over the two STO lines for this platform. Instead:
    #                   * STO1 is the opposite state of 'BrakingCircuit.enable_fets'.
    #                   * STO2 is connected to 'dynamic braking status'?
    ################################################################################################
    class _Everest(Component):
        sto_1                   = 'J2_68'                   # STO1 control line for the motor controller. Set high to enable.
        sto_2                   = 'J2_67'                   # STO2 is controlled by DBB, and connected as GPIO input to pyboard.

        sync_0                  = 'J2_20'
        sync_1                  = 'J2_22'                   # boot/sync_1 pin
        irq                     = 'J2_54'
        reset                   = 'J2_52'

        ############################################################################################
        # @class    SPI
        #
        # @brief    Defines pin routing information for the SPI bus of the Everest servo
        ############################################################################################
        class SPI(object):
            bus_number          = 2                         # Name/ID/Number of target SPI bus.
            chip_select         = 'J2_36'                   # SPI chip select line.

    ################################################################################################
    # @class        _ExternalComm
    #
    # @brief        Defines pin routing information for the main [external] communication bus.
    ################################################################################################
    class _ExternalComm(Component):
        ############################################################################################
        # @class    CAN
        #
        # @brief    Defines pin routing information for the CAN bus used as main communication bus.
        ############################################################################################
        class CAN(object):
            bus_number          = 1                         # Name/ID/Number of target CAN bus.
            rx                  = 'J1_51'                   # CAN receive line used in CAN communication. Connected to output [RXD] line from CAN transceiver.
            tx                  = 'J1_49'                   # CAN transmit line used in CAN communication. Connected to input [TXD] line to CAN transceiver.

    ############################################################################################
    # @class        _ExternalLEDs
    #
    # @brief        Defines pin routing information for the external LEDs provided by CBM.
    #
    # @note         LED requires bit banging in assembly code to communicate. Make sure to set
    #               the protocol type to "Protocol.SK6812".
    ############################################################################################
    class _ExternalLEDs(Component):
        blue                    = 'J2_14'                   # GPIO line for controlling the blue LED.
        green                   = 'J2_10'                   # GPIO line for controlling the green LED.
        red                     = 'J2_12'                   # GPIO line for controlling the red LED.

    # ################################################################################################
    # # @class        _GPIOExpander
    # #
    # # @brief        Defines pin routing information for interacting with the SPI based GPIO
    # #               Expander.
    # ################################################################################################
    # class _GPIOExpander(Component):
    #     ############################################################################################
    #     # @class    SPI
    #     #
    #     # @brief    Defines pin routing information for the SPI based GPIO Expander. It shares the
    #     #           SPI bus with the IMU.
    #     ############################################################################################
    #     class SPI(object):
    #         bus_number          = 1                         # Name/ID/Number of target SPI bus.
    #         chip_select         = 'J7'                      # SPI chip select line.
    #         clock               = 'B3'                      # SPI clock line.
    #         miso                = 'B4'                      # SPI MISO line.
    #         mosi                = 'D7'                      # SPI MOSI line.

    ################################################################################################
    # @class        _IMU
    #
    # @brief        Defines pin routing information for interacting with the external IMU chip.
    ################################################################################################
    class _IMU(Component):
        bootloader_mode_select  = 'J2_58'                   # Bootloader mode select pin.
        data_is_ready           = 'J2_62'                   # Input pin used to inform us that the IMU chip has data ready to be sampled (queried).
        protocol_select_pin_0   = 'J2_18'                   # Protocol select pin 0 [also used to wake processor in SPI mode].
        protocol_select_pin_1   = 'J2_56'                   # Protocol select pin 1.
        reset                   = 'J2_16'                   # Reset pin (active low).

        ############################################################################################
        # @class    SPI
        #
        # @brief    Defines pin routing information for the SPI bus on the BNO085 IMU.
        ############################################################################################
        class SPI(object):
            bus_number          = 1                         # Name/ID/Number of target SPI bus.
            chip_select         = 'J2_46'                   # SPI chip select line.

    ############################################################################################
    # @class        _InternalLEDs
    #
    # @brief        Defines pin routing information for the on-board LEDs provided with the
    #               active MicroPython development board.
    ############################################################################################
    class _InternalLEDs(Component):
        blue                    = 'LED_RED'                 # ID number for controlling the blue LED.
        green                   = 'LED_GREEN'               # ID number for controlling the green LED.
        red                     = 'LED_BLUE'                # ID number for controlling the red LED.

    ################################################################################################
    # @class        _ModeSelectionSwitch
    #
    # @brief        Defines pin routing information for interacting with the rotary position switch.
    #
    # @details      Mode selection is done via an external 10 position switch. It provides an analog
    #               value (line_in) that breaks down as follows:
    #                   * Position-0:   3.300 V
    #                   * Position-1:   1.800 V
    #                   * Position-2:   2.031 V
    #                   * Position-3:   1.342 V
    #                   * Position-4:   2.383 V
    #                   * Position-5:   1.488 V
    #                   * Position-6:   1.642 V
    #                   * Position-7:   1.161 V
    #                   * Position-8:   0.943 V
    #                   * Position-9:   0.762 V
    ################################################################################################
    class _ModeSelectionSwitch(Component):
        line_in                 = 'J2_79'                   # Analog input line to same to read state of rotary switch.

    ################################################################################################
    # @class        _QwiicConnector
    #
    # @brief        Defines pin routing information for interacting with peripherals attached to the
    #               system via the external Qwiic connector.
    ################################################################################################
    class _QwiicConnector(Component):

        ############################################################################################
        # @class    I2C
        #
        # @brief    Defines pin routing information for the I2C bus connected to the Qwiic connector.
        #
        # @note     No address information is tracked here as the remote device address is not a
        #           fixed value.
        ############################################################################################
        class I2C(object):
            bus_number          = 3                         # Name/ID/Number of target I2C bus.
            scl                 = 'J1_46'                   # SCL pin for I2C communication.
            sda                 = 'J1_44'                   # SDA pin for I2C communication.

    # ################################################################################################
    # # @class        _SPIExternal
    # #
    # # @brief        Defines pin routing information for interacting with the external SPI option.
    # ################################################################################################
    # class _SPIExternal(Component):
    #     irq                     = 'C7'

    #     ############################################################################################
    #     # @class    SPI
    #     #
    #     # @brief    Defines pin routing information for the SPI bus related to the external SPI
    #     #           option.
    #     ############################################################################################
    #     class SPI(object):
    #         bus_number          = 1                          # Name/ID/Number of target SPI bus.
    #         chip_select         = 'J10'                     # SPI chip select line.
    #         clock               = 'B3'                      # SPI clock line.
    #         miso                = 'B4'                      # SPI MISO line.
    #         mosi                = 'D7'                      # SPI MOSI line.

    ################################################################################################
    # @class        _TimingOutput
    #
    # @brief        Defines pin routing information for interacting with a GPIO used to time
    #               sections of code
    #
    #               NOTE: pin used is allocated for EXT-SPI-IRQ on connector J4, which allows for
    #               additional SPI peripherals to be connected to SPI[3] in the future
    ################################################################################################
    class _TimingOutput(Component):
        line_out                = 'J1_33'

####################################################################################################
# @class            UPythonLLCtrl_vB_3_0
#
# @brief            Aggregates all hardware specific routing and wiring
#                   information for the Portenta lower limb controller PCB used
#                   in Fox Knee alpha 1.
#
# @details          Routing references generally use the J[1|2]_[0-9]+ notation
#                   for pin references, which correspond to the pinouts of the
#                   two high-density connectors on the Portenta H7.
#
# @warning          The decorator used ensures only a single instance of this
#                   class is used across all Python code (singleton
#                   implementation).
####################################################################################################
@decorators.singleton
class UPythonLLCtrl_vB_3_0(object):

    # ==============================================================================================
    # Define identifying information for the platform this class targets. Breaks down the board
    # name and version information into the variables below.
    # ----------------------------------------------------------------------------------------------
    board_name                              = "upython-ll-ctrl"          # Official name of PCB
    board_revision                          = "B"                        # Revision for PCB (A, B, ...)
    board_major                             = 3                          # Major number for PCB
    board_minor                             = 0                          # Minor number for PCB

    ################################################################################################
    # @brief        Initialization function for class.
    #
    # @details      Instantiates all nested/inner classes and assigns a default protocol to each
    #               component. By default, all components will be disabled and callers can both
    #               override the default protocols and enable the components they wish to utilize.
    #
    # @param[in]    self                Object pointer to class instance.
    ################################################################################################
    def __init__(self):
        # Create an instance of all first-level nested/inner classes within this class. Doing so in
        # the following manner allows us to make each inner class of type 'Component' and have
        # external callers modify the state/value of each 'Component' (without instantiation we
        # cannot effectively enable, disable, set protocol values, etc.).
        self.BrakingCircuit                 = self._BrakingCircuit()
        self.Buzzer                         = self._Buzzer()
        self.ESCON                          = self._ESCON()
        self.Everest                        = self._Everest()
        self.ExternalComm                   = self._ExternalComm()
        self.ExternalLEDs                   = self._ExternalLEDs()
        self.ForceTorqueSensor              = self._ForceTorqueSensor()
        self.IMU                            = self._IMU()
        self.InternalLEDs                   = self._InternalLEDs()
        self.ModeSelectionSwitch            = self._ModeSelectionSwitch()
        self.PeripheralPower                = self._PeripheralPower()
        self.QwiicConnector                 = self._QwiicConnector()
        self.TimingOutput                   = self._TimingOutput()

        # Select the desired protocol to use for any and all component objects within this class.
        self.BrakingCircuit.protocol        = Protocol.GPIO
        self.Buzzer.protocol                = Protocol.GPIO
        self.Everest.protocol               = Protocol.SPI
        self.ExternalComm.protocol          = Protocol.CAN
        self.ExternalLEDs.protocol          = Protocol.GPIO
        self.IMU.protocol                   = Protocol.SPI
        self.InternalLEDs.protocol          = Protocol.GPIO
        self.ModeSelectionSwitch.protocol   = Protocol.ANALOG
        self.PeripheralPower.protocol       = Protocol.GPIO

    ################################################################################################
    # @class        _BrakingCircuit
    #
    # @brief        Defines pin routing information for the "dynamic braking circuit".
    ################################################################################################
    class _BrakingCircuit(Component):
        enable_fets             = 'J2_65'                   # Digital output to enable braking mode.

        ############################################################################################
        # @class    PWM
        #
        # @brief    Defines pin routing information for the PWM output pin driving the braking
        #           circuit to control the rate of braking on the main motor (faster = more braking
        #           and stiffer joint).
        #
        # @note     Only relevant if the 'enable_fets' line is active.
        ############################################################################################
        class PWM(object):
            pin                 = 'J2_60'                   # Line to drive for PWM output.
            invert              = False                     # Invert the PWM signal.

    ############################################################################################
    # @class    _Buzzer
    #
    # @brief    Defines pin routing information for driving the external buzzer via a DAC pin.
    ############################################################################################
    class _Buzzer(Component):
        line_out                = 'J2_61'                   # Line to drive when controlling the external buzzer

    ################################################################################################
    # @class        _ESCON
    #
    # @brief        Dummy entry. Board has no ESCON support.
    ################################################################################################
    class _ESCON(Component):
        pass

    ################################################################################################
    # @class        _Everest
    #
    # @brief        Defines pin routing information for interacting with the Novanta Everest motor
    #               controller.
    #
    # @note         There is no direct control over the two STO lines for this platform. Instead:
    #                   * STO1 is the opposite state of 'BrakingCircuit.enable_fets'.
    #                   * STO2 is connected to 'dynamic braking status'
    ################################################################################################
    class _Everest(Component):
        sto_1                   = 'J2_68'                   # STO1 line for motor controller. Set high to enable.
        sto_2                   = 'J2_67'                   # STO2 is controlled by DBB, and connected as GPIO input.

        sync_0                  = 'J2_20'
        sync_1                  = 'J2_22'                   # boot/sync_1 pin
        irq                     = 'J2_54'
        reset                   = 'J2_52'

        ############################################################################################
        # @class    SPI
        #
        # @brief    Defines pin routing information for the SPI bus of the Everest servo
        ############################################################################################
        class SPI(object):
            bus_number          = 2                         # Name/ID/Number of target SPI bus.
            chip_select         = 'J2_36'                   # SPI chip select line.

    ################################################################################################
    # @class        _ExternalComm
    #
    # @brief        Defines pin routing information for the main [external] communication bus.
    ################################################################################################
    class _ExternalComm(Component):
        ############################################################################################
        # @class    CAN
        #
        # @brief    Defines pin routing information for the CAN bus used as main communication bus.
        ############################################################################################
        class CAN(object):
            bus_number          = 1                         # Name/ID/Number of target CAN bus.
            rx                  = 'J1_51'                   # CAN receive line.
            tx                  = 'J1_49'                   # CAN transmit line.

    ############################################################################################
    # @class        _ExternalLEDs
    #
    # @brief        Defines pin routing information for the external LEDs provided by CBM.
    #
    # @note         LED requires bit banging in assembly code to communicate. Make sure to set
    #               the protocol type to "Protocol.SK6812".
    ############################################################################################
    class _ExternalLEDs(Component):
        blue                    = 'J2_14'                   # GPIO line for blue LED.
        green                   = 'J2_10'                   # GPIO line for green LED.
        red                     = 'J2_12'                   # GPIO line for red LED.

    ############################################################################################
    # @class        _ForceTorqueSensor
    #
    # @brief        Defines pin routing information for the Force Torque Sensor.

    ############################################################################################
    class _ForceTorqueSensor(Component):
        data_ready             = 'J2_8'
        power_reset            = 'J1_46'
        start                  = 'J1_44'

        ############################################################################################
        # @class    SPI
        #
        # @brief    Defines pin routing information for the SPI bus of the Force Torque Sensor.
        ############################################################################################
        class SPI(object):
            bus_number          = 1
            chip_select         = 'J2_4'

    ################################################################################################
    # @class        _IMU
    #
    # @brief        Defines pin routing information for interacting with the external IMU chip.
    ################################################################################################
    class _IMU(Component):
        bootloader_mode_select  = 'J2_58'                   # Bootloader mode select pin.
        data_is_ready           = 'J2_62'                   # IMU chip has data ready to be sampled.
        protocol_select_pin_0   = 'J2_18'                   # Protocol select pin 0. Also wake pin.
        protocol_select_pin_1   = 'J2_56'                   # Protocol select pin 1.
        reset                   = 'J2_16'                   # Reset pin (active low).

        ############################################################################################
        # @class    SPI
        #
        # @brief    Defines pin routing information for the SPI bus on the BNO085 IMU.
        ############################################################################################
        class SPI(object):
            bus_number          = 1                         # Name/ID/Number of target SPI bus.
            chip_select         = 'J2_46'                   # SPI chip select line.

    ############################################################################################
    # @class        _InternalLEDs
    #
    # @brief        Defines pin routing information for the on-board LEDs provided with the
    #               active MicroPython development board.
    ############################################################################################
    class _InternalLEDs(Component):
        blue                    = 'LED_RED'                 # ID number for controlling the blue LED.
        green                   = 'LED_GREEN'               # ID number for controlling the green LED.
        red                     = 'LED_BLUE'                # ID number for controlling the red LED.

    ################################################################################################
    # @class        _ModeSelectionSwitch
    #
    # @brief        Defines pin routing information for interacting with the rotary position switch.
    #
    # @details      In earlier controller boards, mode selection was done via an external 10
    #               position switch, providing an analog value (line_in) as follows:
    #                   * Position-0:   3.300 V
    #                   * Position-1:   1.800 V
    #                   * Position-2:   2.031 V
    #                   * Position-3:   1.342 V
    #                   * Position-4:   2.383 V
    #                   * Position-5:   1.488 V
    #                   * Position-6:   1.642 V
    #                   * Position-7:   1.161 V
    #                   * Position-8:   0.943 V
    #                   * Position-9:   0.762 V
    #
    #               In this version, the mode selection switch is replaced with a dip switch
    #               that switches between positions 0 and 8 above. A second dip switch is
    #               added for future use which simply utilizes a GPIO pin.
    ################################################################################################
    class _ModeSelectionSwitch(Component):
        line_in                 = 'J2_79'                   # Analog input line
        variant_in              = 'J2_77'                   # GPIO input line

    ################################################################################################
    # @class        _PeripheralPower
    #
    # @brief        Defines pin routing information for interacting with peripheral power control
    #               lines.
    ################################################################################################
    class _PeripheralPower(Component):
        external_spi_power      = 'J2_28'                   # Force Torque Sensor Power

    ################################################################################################
    # @class        _QwiicConnector
    #
    # @brief        Dummy entry. Board has no Qwiic connector support.
    ################################################################################################
    class _QwiicConnector(Component):
        pass

    ################################################################################################
    # @class        _TimingOutput
    #
    # @brief        Defines pin routing information for interacting with a GPIO used to time
    #               sections of code
    #
    #               NOTE: pin used is allocated for EXT-SPI-IRQ on connector J4, which allows for
    #               additional SPI peripherals to be connected to SPI[3] in the future
    ################################################################################################
    class _TimingOutput(Component):
        line_out                = 'J1_33'                   # PA9 on Portenta H7 silkscreen