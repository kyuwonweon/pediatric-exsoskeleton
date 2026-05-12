import sys
import micropython  # type: ignore
import pyb  # type: ignore
import stm  # type: ignore
from uctypes import addressof  # type: ignore

import array

from . import upy_float
from .const_dict import CONST_DICT
from .novanta_constants import NovantaConstants as NOV


# -----------------------------------------------------------------------------
#
#   CLASS EVEREST()
#
# -----------------------------------------------------------------------------
class Everest:
    """Represents the Everest motor servo controller.

    This class provides methods for controlling and communicating
    with the Novanta motor servo controller. It includes functionalities
    like sending/receiving dataframes, configuring registers, and controlling
    the motor in cyclic mode.

    Parameters:
        spi_object (upySPI): An SPI object for communication.
        reset_pin_id (str): The pin used for resetting the controller.
        boot_sync1_pin_id (str): The pin used for boot synchronization.
        select_pin_id (str): The pin used to select the controller.
        interrupt_pin_id (str): The pin used for interrupts.
        config_dict (dict): Configuration parameters.
        cyclic_pico_registers (list of int): Registers for cyclic PICO
            communication.
        cyclic_poci_registers (list of int): Registers for cyclic POCI
            communication.
        use_hardware_crc (bool): A flag indicating if hardware CRC is used.
        verbose (bool): A flag to enable verbose output.

    Attributes:
        .spi (object): An SPI object for communication.
        .verbose (boolean): A flag to enable verbose output.
        .boot_sync1_pin_id (string): The pin used for boot synchronization.
        .use_hardware_crc (boolean): A flag indicating if hardware CRC is used.
        .float_holder(float_array): A float object.
        .communication_mode (int): The communication mode.
        .SPI_ready (bool): A flag indicating if the SPI is ready.
        .channel_config (Channel): A configuration channel.
        ._IDLE_CONFIG_BUFFER (bytearray): An idle send buffer.
        .channel_pico (Channel): A PICO channel.
        .channel_poci (Channel): A POCI channel.
        .pico_cyclic_dict (dict): A dictionary of cyclic PICO registers.
        .poci_cyclic_dict (dict): A dictionary of cyclic POCI registers.
        .cyclic_size (int): The size of the cyclic buffer.
        ._RESET_TIMEOUT (int): The reset timeout.
    """

    # -------------------------------------------------------------------------
    #   CLASS CONSTANTS
    # -------------------------------------------------------------------------
    #
    # Constants that are defined by Novanta are in the NOV dictionary,
    # constants defined by us are defined here

    _RESET_TIMEOUT = 5000  # (ms)

    #
    #   SERVO STATE MACHINE IDENTIFIERS (CiA402)
    #
    STATE_NOT_READY_TO_SWITCH_ON = 1
    STATE_SWITCH_ON_DISABLED = 2
    STATE_READY_TO_SWITCH_ON = 3
    STATE_SWITCHED_ON = 4
    STATE_OPERATION_ENABLED = 5
    STATE_QUICK_STOP_ACTIVE = 6
    STATE_FAULT_REACTION_ACTIVE = 7
    STATE_FAULT = 8

    #
    #   COMMUNICATION MODE IDENTIFIERS
    #
    _COMMUNICATION_MODE_BOOT = 0
    _COMMUNICATION_MODE_CONFIG = 1
    _COMMUNICATION_MODE_CYCLIC = 2

    #
    #   ADDITIONAL REGISTER IDENTIFIERS
    #
    # The following registers aren't defined by name in the Novanta spec,
    # and are referenced by literals in the documentation
    _COMMUNICATION_MODE_STATUS = 0x640
    _CYCLIC_RX_NUM_ENTRIES = 0x650
    _CYCLIC_TX_NUM_ENTRIES = 0x660

    # -------------------------------------------------------------------------
    #   __INIT__()
    # -------------------------------------------------------------------------
    def __init__(
            self,
            spi_object,
            reset_pin_id,
            boot_sync1_pin_id,
            select_pin_id,
            interrupt_pin_id,
            config_dict=None,
            cyclic_pico_registers=None,
            cyclic_poci_registers=None,
            use_hardware_crc=True,
            verbose=False,
    ):

        self.spi = spi_object
        self.verbose = verbose
        self.boot_sync1_pin_id = boot_sync1_pin_id  # pin function changes
        self.use_hardware_crc = use_hardware_crc

        # Create comm channel for config mode and use it for dataframe buffers
        # until cyclic mode is enabled Create an idle send buffer to send when
        # recieving a reply in config mode
        # self.channel_config = Channel(self)
        self.channel_config = Channel(self)
        self._IDLE_CONFIG_BUFFER = micropython.const(
            bytearray(b'\x00\x0e\x00\x00\x00\x00\x00\x00\x00\x00sw')
        )

        # temporary use of a float
        self.float_holder = upy_float.upyFloat(0.0)

        self.communication_mode = self._COMMUNICATION_MODE_BOOT  # needed?
        self.SPI_ready = False

        #
        #   CONFIGURE I/O PINS
        #

        #   boot_sync1 is configured as input for device reset
        #       * high state: normal operation
        #       * low state: enters bootloader
        #   following reset, this pin is changed to be an input. The Everest
        #   "generates a continuous pulse with the base time of the internal
        #   control loops." (from documentation)
        self.boot_sync1_pin = pyb.Pin(
            boot_sync1_pin_id,
            pyb.Pin.OUT_OD,
            pull=pyb.Pin.PULL_UP
        )
        self.reset_pin = pyb.Pin(  # active low
            reset_pin_id,
            pyb.Pin.OUT_OD,
            pull=pyb.Pin.PULL_UP
        )
        self.select_pin = pyb.Pin(  # active low
            select_pin_id,
            pyb.Pin.OUT_OD,
            pull=pyb.Pin.PULL_UP  # docs are ambiguous
        )
        self.select_pin.high()

        self.interrupt_pin = pyb.ExtInt(
            interrupt_pin_id,
            pyb.ExtInt.IRQ_RISING,
            pyb.Pin.PULL_DOWN,
            self.isr
        )

        # Select which CRC function to use. hardware is fast (~7us but may have
        # conflicts?), software is slow (~160 us)
        if use_hardware_crc:
            self.init_crc_arm()
            self.crc16 = self.crc_arm
        else:
            self.crc16 = self.crc_software

        # reset the servo
        if not self.hard_reset():
            return

        # ---------------------------------------------------------------------
        #   SET UP PICO AND POCI CHANNELS
        # ---------------------------------------------------------------------
        self.pico_cyclic_dict = {}
        self.poci_cyclic_dict = {}

        pico_cyclic_size = 0
        poci_cyclic_size = 0

        if cyclic_pico_registers:
            pico_cyclic_size = self.build_cyclic_dict(
                cyclic_pico_registers,
                self.pico_cyclic_dict
            )

        if cyclic_poci_registers:
            poci_cyclic_size = self.build_cyclic_dict(
                cyclic_poci_registers,
                self.poci_cyclic_dict
            )

        self.cyclic_size = max(pico_cyclic_size, poci_cyclic_size)

        self.channel_pico = Channel(
            self,
            channel_type=Channel.PICO
        )
        self.channel_poci = Channel(
            self,
            channel_type=Channel.POCI
        )

        # ---------------------------------------------------------------------
        #   PROCESS CONFIGURATION DICTIONARY
        # ---------------------------------------------------------------------
        if config_dict:
            for register, value in config_dict.items():
                self.set_register(register, value)

        return

    # -------------------------------------------------------------------------
    #   BUILD_CYCLIC_DICT()
    # -------------------------------------------------------------------------
    def build_cyclic_dict(self, registers, cyclic_dict) -> int:
        """Builds a dictionary of cyclic registers.

        This method builds a dictionary of cyclic registers. The dictionary
        contains information about the register name, size, data type, and
        cyclic offset.

        Parameters:
            registers (list): A list of register addresses.
            cyclic_dict (dict): A dictionary to store the cyclic register
                information.

        Returns:
            int: The size of the cyclic buffer.
        """

        cyclic_size = 0

        if isinstance(registers, int):
            registers = [registers]

        for register in registers:
            channel_name = NOV.get_register_name(register)

            register_info = self.get_register_info(register)

            if register_info['cyclic_type'] == NOV.CYCLIC_CONFIG_ONLY:
                print(f'{channel_name} NOT AVAILABLE FOR CYCLIC ACCESS')
                continue

            name = channel_name.lower()
            size = register_info['size']
            data_type = register_info['data_type']
            cyclic_offset = cyclic_size

            cyclic_dict[register] = {
                'name': name,
                'size': size,
                'data_type': data_type,
                'cyclic_offset': cyclic_offset
            }

            cyclic_size += size

        return cyclic_size

    # -------------------------------------------------------------------------
    #   HARD_RESET()
    # -------------------------------------------------------------------------
    def hard_reset(self):
        """Performs a hard reset of the servo controller.

        This method performs a hard reset of the servo controller. It sets the
        communication mode to bootloader mode and resets the controller. The
        method waits for the controller to be ready to communicate before
        returning.

        Parameters:
            None

        Returns:
            bool: True if the reset is successful, False otherwise.
        """

        timeout = self._RESET_TIMEOUT
        self.communication_mode = self._COMMUNICATION_MODE_BOOT

        if self.verbose:
            print()
            print('-------------------------------')
            print('NOVANTA EVEREST HARD RESET')

        self.interrupt_pin.disable()  # disable interrupt
        self.SPI_ready = False

        # this pin used to select bootloader/normal operation at boot (output)
        # then becomes watchdog pin from Ing following boot (input)
        self.boot_sync1_pin = pyb.Pin(
            self.boot_sync1_pin_id,
            pyb.Pin.OUT_OD,
            pull=pyb.Pin.PULL_UP
        )

        # Start-up process defined in "MCB start-up sequence" documentation
        self.reset_pin.high()
        self.boot_sync1_pin.low()
        pyb.udelay(50)

        self.reset_pin.low()
        pyb.udelay(75)
        self.boot_sync1_pin.high()
        self.interrupt_pin.enable()  # re-enable interrupt

        pyb.udelay(75)
        self.reset_pin.high()

        # wait for servo to go through boot process, which ends with the irq
        # pin being pulled high
        print('Booting Novanta OS', end='')
        while not self.SPI_ready and timeout:
            pyb.delay(1)
            timeout = timeout - 1
            if not timeout % 250:
                print('.', end='')
        print('.')

        # change pin function for sync1 from output to input
        self.boot_sync1_pin = pyb.Pin(
            self.boot_sync1_pin_id,
            pyb.Pin.IN,
            pull=pyb.Pin.PULL_DOWN  # ?
        )

        # if reset properly occurred without timeout expiring
        if timeout:
            self.communication_mode = self._COMMUNICATION_MODE_CONFIG
            if self.verbose:
                print(
                    'Reset confirmed after ',
                    self._RESET_TIMEOUT - timeout,
                    ' ms'
                )

        else:
            if self.verbose:
                print('ERROR: Reset did not occur')
                return False
        if self.verbose:
            print('-------------------------------')

        return True

    # -------------------------------------------------------------------------
    #   SET_REGISTER()
    # -------------------------------------------------------------------------
    def set_register(self, register, value, verbose=True):
        """Sets the value of a specific register.

        This method is used to configure a specific servo register with a given
        value. Note that the value writen to the register is not checked for
        validity. It is the responsibility of the user to ensure that the value
        is of the correct type and within the valid range for the register.

        Parameters:
            register (int): The register address to write to. value (int or
            float): The value to be written to the register. verbose (bool): If
            True, provides detailed logging output.

        Returns:
            None

        Raises:
            ValueError: If the register address or value is invalid.
            CommunicationError: If there is a problem communicating with the
            servo.
        """
        register_name = NOV.get_register_name(register)

        register_info = self.get_register_info(register)
        data_type = register_info['data_type']

        if data_type == NOV.TYPE_FLOAT:
            initial_value = self.read_float(register)
            self.write_float(register, value)
            final_value = self.read_float(register)

        elif (
                data_type == NOV.TYPE_UINT16
                or data_type == NOV.TYPE_UINT32
        ):
            initial_value = self.read_uint(register)
            self.write_uint(register, value)
            final_value = self.read_uint(register)

        elif (
                data_type == NOV.TYPE_INT16
                or data_type == NOV.TYPE_INT32
        ):
            initial_value = self.read_int(register)
            self.write_int(register, value)
            final_value = self.read_int(register)

        if verbose:
            print(f'Writing {str(value):>8} to {register_name:<60} ', end='')
            print(f'[0x{register:03X}]: {str(initial_value):>8} -> ', end='')
            print(f'{str(final_value):<8} ', end='')
            if final_value == value:
                if final_value == initial_value:
                    print('🟰')
                else:
                    # print('')
                    print('👍')
            else:
                # print('!')
                print('🛑')

        return

    # -------------------------------------------------------------------------
    #   ENABLE_CYCLIC_OPERATION()
    # -------------------------------------------------------------------------
    def enable_cyclic_mode(self):
        """Enables cyclic mode for the servo controller.

        This method enables cyclic mode for the servo controller. It sets the
        communication mode to cyclic mode and writes a value of 2 to the
        communication mode register.

        Parameters:
            None

        Returns:
            None
        """
        self.write_uint(0x640, 2)  # documentation uses literals for 0x640
        self.communication_mode = self._COMMUNICATION_MODE_CYCLIC
        if self.verbose:
            print('Everest CYCLIC mode enabled')

    # -------------------------------------------------------------------------
    #   DISABLE_CYCLIC_OPERATION()
    # -------------------------------------------------------------------------
    def disable_cyclic_mode(self):
        """Disables cyclic mode for the servo controller.

        This method disables cyclic mode for the servo controller. It sets the
        communication mode to configuration mode and writes a value of 1 to the
        communication mode register.

        Parameters:
            None

        Returns:
            None
        """
        self.write_uint(0x640, 1)
        self.communication_mode = self._COMMUNICATION_MODE_CONFIG

    # -------------------------------------------------------------------------
    #   DATA FRAME FORMAT
    # -------------------------------------------------------------------------
    #   The 'set_[config|cyclic]_buffer():' methods build data frames. Data
    #   frame format is as follows:
    #
    #   address:              12 bits ─┐
    #   command:               3 bits  ├─▶ Header (1 word)
    #   pending frame:         1 bit  ─┘
    #   config data:           4 words
    #   cyclic data:           0 - 32 words (optional)
    #   CRC:                   1 word
    #
    #   If the config data field is fragmentend (needs more than 4 words),
    #   multiple frames are sent. Pending frame = 0 indicates the current frame
    #   is the last frame (or only frame) in sequence.
    # -------------------------------------------------------------------------

    # -------------------------------------------------------------------------
    #   SET_CONFIG_BUFFER()
    # -------------------------------------------------------------------------
    def set_config_buffer(
            self, address, command, pending,
            data=0,
            use_data_buffer=False
    ):
        """Sets the configuration buffer for the servo controller.

        This method is used to build a data frame for the servo controller in
        configuration mode. The data frame consists of a header, configuration
        data, and a CRC. The header contains the address, command, and a pending
        frame bit. The configuration data is a 4-word array. The CRC is a 1-word
        value.

        Parameters:
            address (int): The address of the register to access.
            command (int): The command to send to the register.
            pending (int): A flag indicating if the frame is pending.
            data (int): The data to send to the register.
            use_data_buffer (bool): A flag indicating if the data buffer should
                be used.

        Returns:
            None
        """

        if self.communication_mode == self._COMMUNICATION_MODE_CYCLIC:
            header_buffer = self.channel_pico.header_mv
            data_buffer = self.channel_pico.config_data_mv
        else:
            header_buffer = self.channel_config.header_mv
            data_buffer = self.channel_config.config_data_mv

        header_buffer[0] = (address & 0xFF0) >> 4
        header_buffer[1] = (
                ((address & 0xF) << 4)
                + ((command & 0b111) << 1)
                + (pending & 0b1)
        )

        if not use_data_buffer:
            data_buffer[0] = (data >> 56) & 0xFF
            data_buffer[1] = (data >> 48) & 0xFF
            data_buffer[2] = (data >> 40) & 0xFF
            data_buffer[3] = (data >> 32) & 0xFF
            data_buffer[4] = (data >> 24) & 0xFF
            data_buffer[5] = (data >> 16) & 0xFF
            data_buffer[6] = (data >> 8) & 0xFF
            data_buffer[7] = (data) & 0xFF

    # -------------------------------------------------------------------------
    #   SEND_RECEIVE_CALL_RESPONSE()
    # -------------------------------------------------------------------------
    def send_receive_call_response(self):
        """Sends and receives a call response from the servo controller.

        This method sends a data frame to the servo controller and receives a
        response. The response is checked for a CRC error. If the communication
        mode is configuration mode, the method sends and receives a single
        frame. If the communication mode is cyclic mode, the method sends and
        receives multiple times until a successful reply is received.

        Parameters:
            None

        Returns:
            None
        """
        if self.communication_mode == self._COMMUNICATION_MODE_CONFIG:
            crc = self.crc16(
                self.channel_config.CRC_data_mv,
                self.channel_config.CRC_data_len
            )
            self.channel_config.set_crc(crc)

            count = 0
            while not self.SPI_ready:
                count += 1
            self.SPI_ready = False

            self.select_pin.low()
            self.spi.write(self.channel_config.buffer)
            self.select_pin.high()

            count = 0
            while not self.SPI_ready:
                count += 1
            self.SPI_ready = False

            self.select_pin.low()
            self.spi.write_readinto(
                self._IDLE_CONFIG_BUFFER,
                self.channel_config.buffer
            )
            self.select_pin.high()

            if (
                    self.crc16(
                        self.channel_config.buffer,
                        self.channel_config.CRC_data_len + 2
                    )
                    and self.verbose
            ):
                print('EVEREST CRC ERROR ON REPLY')

        elif self.communication_mode == self._COMMUNICATION_MODE_CYCLIC:
            self.send_receive_cyclic
            command = (self.channel_poci.buffer_mv[1] >> 1) & 0b111
            while command != NOV.CMD_ACK:
                self.send_receive_cyclic
                command = (self.channel_poci.buffer_mv[1] >> 1) & 0b111
            self.channel_pico.set_idle()

    # -------------------------------------------------------------------------
    #   SEND_RECEIVE_CYCLIC()
    # -------------------------------------------------------------------------
    @micropython.native
    def send_receive_cyclic(self):
        """Sends and receives a data frame in cyclic mode.

        This method exchanges a data frame with the servo controller in cyclic
        mode. It performs duplex communication by sending a PICO channel and
        receiving a POCI channel.

        Parameters:
            None

        Returns:
            None
        """
        crc = self.crc16(
            self.channel_pico.CRC_data_mv,
            self.channel_pico.CRC_data_len
        )
        self.channel_pico.set_crc(crc)

        count = 0
        while not self.SPI_ready:
            count += 1
        self.SPI_ready = False

        # self.select_pin.low()
        self.select_pin(0)
        self.spi.write_readinto(
            self.channel_pico.buffer_mv,
            self.channel_poci.buffer_mv
        )
        self.select_pin(1)

        # Removed CRC check for speed 2023-03-30
        # check CRC in response
        # if (
        #     self.crc16(
        #         self.channel_poci.buffer,
        #         self.channel_poci.CRC_data_len + 2
        #     )
        # ):
        #     # TODO: raise error
        #     pass

        # poci_command = (self.channel_poci.header_mv[1] >> 1) & 0b111
        # if poci_command == NOV.CMD_IDLE:
        return

    # -------------------------------------------------------------------------
    #   STORE_AXIS_PARAMETERS()
    # -------------------------------------------------------------------------
    def store_axis_parameters(self):
        """Stores the axis parameters to the Everest flash memory.

        This method stores the axis parameters to the Everest flash memory. It
        writes a value of 1 to the STORE_ALL register to save all parameters to
        the flash memory.

        Parameters:
            None

        Returns:
            None
        """

        # store axis parameters
        self.write_uint(NOV.STORE_ALL, NOV.STORE_ALL_PASSWORD)
        print('Storing parameters to Everest flash ', end='')
        for _ in range(8):
            pyb.delay(500)
            print('.', end='')
        print('')
        print('Parameters stored')
        return

    # -------------------------------------------------------------------------
    #   READ_FLOAT()
    # -------------------------------------------------------------------------
    @micropython.native
    def read_float(self, address: int) -> float:
        """Reads a float value from a specific register.

        This method reads a float value from a specific register. It sets the
        configuration buffer with the address and command to read the register.
        The method then sends and receives the call response from the servo
        controller.

        Parameters:
            address (int): The address of the register to read.

        Returns:
            float: The float value read from the register.
        """
        self.set_config_buffer(address, NOV.CMD_READ, False)
        self.send_receive_call_response()

        if self.communication_mode == self._COMMUNICATION_MODE_CYCLIC:
            return self.channel_poci.float_holder.val
        elif self.communication_mode == self._COMMUNICATION_MODE_CONFIG:
            return self.channel_config.float_from_config_data()
        else:
            return 0

    # -------------------------------------------------------------------------
    #   WRITE_FLOAT()
    # -------------------------------------------------------------------------
    @micropython.native
    def write_float(self, address: int, value: float) -> None:
        """Writes a float value to a specific register.

        This method writes a float value to a specific register. It sets the
        configuration buffer with the address, command, and value to write to
        the register. The method then sends and receives the call response from
        the servo controller.

        Parameters:
            address (int): The address of the register to write.
            value (float): The float value to write to the register.

        Returns:
            None
        """

        if self.communication_mode == self._COMMUNICATION_MODE_CYCLIC:
            self.channel_pico.float_to_config_data(value)
        elif self.communication_mode == self._COMMUNICATION_MODE_CONFIG:
            self.channel_config.float_to_config_data(value)
        self.set_config_buffer(address, NOV.CMD_WRITE, 0, use_data_buffer=True)
        self.send_receive_call_response()

    # -------------------------------------------------------------------------
    #   READ_UINT()
    # -------------------------------------------------------------------------
    @micropython.native
    def read_uint(self, address: int) -> int:
        """Reads an unsigned integer value from a specific register.

        This method reads an unsigned integer value from a specific register. It
        sets the configuration buffer with the address and command to read the
        register. The method then sends and receives the call response from the
        servo controller.

        Parameters:
            address (int): The address of the register to read.

        Returns:
            int: The unsigned integer value read from the register.
        """

        self.set_config_buffer(address, NOV.CMD_READ, False)
        self.send_receive_call_response()

        if self.communication_mode == self._COMMUNICATION_MODE_CYCLIC:
            data_buffer = self.channel_poci.config_data_mv
        elif self.communication_mode == self._COMMUNICATION_MODE_CONFIG:
            # TODO: use this system for all reads, including read_int()
            return self.channel_config.int_from_config_data()
        return (
                data_buffer[2] << 24
                | data_buffer[3] << 16
                | data_buffer[0] << 8
                | data_buffer[1]
        )

    # -------------------------------------------------------------------------
    #   WRITE_UINT()
    # -------------------------------------------------------------------------
    def write_uint(self, address, value):
        """Writes an unsigned integer value to a specific register.

        This method writes an unsigned integer value to a specific register. It
        sets the configuration buffer with the address, command, and value to
        write to the register. The method then sends and receives the call
        response from the servo controller.

        Parameters:
            address (int): The address of the register to write.
            value (int): The unsigned integer value to write to the register.

        Returns:
            None
        """
        self.set_config_buffer(
            address,
            NOV.CMD_WRITE,
            False,
            data=((value & 0xFFFF) << 48) | (((value >> 16) & 0xFFFF) << 32)
        )
        self.send_receive_call_response()

    # -------------------------------------------------------------------------
    #   READ_INT()
    # -------------------------------------------------------------------------
    def read_int(self, address):
        """Reads a signed integer value from a specific register.

        This method reads a signed integer value from a specific register. It
        sets the configuration buffer with the address and command to read the
        register. The method then sends and receives the call response from the
        servo controller.

        Parameters:
            address (int): The address of the register to read.

        Returns:
            int: The signed integer value read from the register.
        """
        self.set_config_buffer(address, NOV.CMD_READ, False)
        self.send_receive_call_response()

        if self.communication_mode == self._COMMUNICATION_MODE_CYCLIC:
            data_buffer = self.channel_poci.config_data_mv
        elif self.communication_mode == self._COMMUNICATION_MODE_CONFIG:
            data_buffer = self.channel_config.config_data_mv

        value = (
                data_buffer[2] << 24
                | data_buffer[3] << 16
                | data_buffer[0] << 8
                | data_buffer[1]
        )
        if data_buffer[2] & 0x80:
            value = value - 0x100000000
        return value

    # -------------------------------------------------------------------------
    #   WRITE_INT()
    # -------------------------------------------------------------------------
    def write_int(self, address, value):
        """Writes a signed integer value to a specific register.

        This method writes a signed integer value to a specific register. It
        sets the configuration buffer with the address, command, and value to
        write to the register. The method then sends and receives the call
        response from the servo controller.

        Parameters:
            address (int): The address of the register to write.
            value (int): The signed integer value to write to the register.

        Returns:
            None
        """
        self.set_config_buffer(
            address,
            NOV.CMD_WRITE,
            False,
            data=((value & 0xFFFF) << 48) | (((value >> 16) & 0xFFFF) << 32)
        )
        self.send_receive_call_response()

    # -------------------------------------------------------------------------
    #   SET_JOINT_OFFSET()
    # -------------------------------------------------------------------------
    def set_joint_offset(self):
        """Sets the joint encoder offset to zero the sensor.

        This method sets the joint encoder offset to zero. It reads the current
        value of the joint encoder position and sets the offset to its negative
        value.

        Parameters:
            None

        Returns:
            None
        """
        self.write_uint(NOV.PRIMARY_ABSOLUTE_SLAVE_1_POSITION_OFFSET, 0)
        pyb.delay(100)
        value = 0 - self.read_int(NOV.PRIMARY_ABSOLUTE_SLAVE_1_POSITION)
        pyb.delay(100)
        self.write_uint(NOV.PRIMARY_ABSOLUTE_SLAVE_1_POSITION_OFFSET, value)
        pyb.delay(100)
        print('Joint Encoder offset set to', value)
        self.store_axis_parameters()

    # -------------------------------------------------------------------------
    #   CLEAR_BUFFER()
    # -------------------------------------------------------------------------
    def clear_buffer(self, buffer):
        """Clears the buffer by setting all values to zero.

        This method clears the buffer by setting all values to zero. It is
        faster than using a for loop to set each value to zero.

        Parameters:
            buffer (bytearray): The buffer to clear.

        Returns:
            None
        """
        # buffer = b'\x00' * len(buffer)        # faster, but non-ISR-safe
        for i in range(len(buffer)):
            buffer[i] = 0x0

    # -------------------------------------------------------------------------
    #   ISR()
    # -------------------------------------------------------------------------
    # @micropython.native
    def isr(self, line) -> None:
        """Interrupt service routine for the Everest servo controller.

        This method is the interrupt service routine for the Everest servo
        controller. It is called when the interrupt pin is triggered,
        signalling that the servo is no longer in a busy state. The method sets
        the SPI ready flag to True to indicate that the servo is ready to
        receive data.

        Parameters:
            line (int): The line number of the interrupt pin.

        Returns:
            None
        """
        self.SPI_ready = True

    # -------------------------------------------------------------------------
    #   REPORT_STATE()
    # -------------------------------------------------------------------------
    def report_state(self):
        """Reports the current state of the servo controller.

        This method reports the current state of the servo controller. It reads
        the status word register and determines the state of the controller
        based on the status word value.

        Parameters:
            None

        Returns:
            None
        """
        self.set_config_buffer(NOV.STATUS_WORD, NOV.CMD_READ, 0, 0)
        self.send_receive_call_response()
        buf = self.channel_config.buffer
        status = (buf[2] << 8) | buf[3]
        print('STATUS =', bin(status))
        state = self.state_from_status_word(status)

        if state == NOV.STATE_NOT_READY_TO_SWITCH_ON:
            status_string = 'NOT READY TO SWITCH ON'
        elif state == NOV.STATE_SWITCH_ON_DISABLED:
            status_string = 'SWITCH_ON_DISABLED'
        elif state == NOV.STATE_READY_TO_SWITCH_ON:
            status_string = 'READY TO SWITCH ON'
        elif state == NOV.STATE_SWITCHED_ON:
            status_string = 'SWITCHED_ON'
        elif state == NOV.STATE_OPERATION_ENABLE:
            status_string = 'OPERATION_ENABLE'
        elif state == NOV.STATE_QUICK_STOP:
            status_string = 'QUICK STOP ACTIVE'
        elif state == NOV.STATE_FAULT_REACTION:
            status_string = 'FAULT REACTION'
        elif state == NOV.STATE_FAULT:
            status_string = 'FAULT'
        else:
            status_string = 'INVALID STATE'
        print('STATE =', status_string)

    # -------------------------------------------------------------------------
    #   REPORT_LAST_ERROR()
    # -------------------------------------------------------------------------
    def report_last_error(self):
        """Reports the last error code from the servo controller.

        This method reports the last error code from the servo controller. It
        reads the last error register and prints the error code.

        Parameters:
            None

        Returns:
            int: The last error code value.
        """
        value = self.read_uint(NOV.LAST_ERROR)
        return value

    # -------------------------------------------------------------------------
    #   STATE_FROM_STATUS_WORD()
    # -------------------------------------------------------------------------
    def state_from_status_word(self, status):
        """Determines the state of the servo controller from the status word.

        This method determines the state of the servo controller from the status
        word. It reads the status word value and masks the relevant bits to
        determine the state of the controller.

        Parameters:
            status (int): The status word value.

        Returns:
            int: The state of the controller.
        """
        status = status & 0b1101111  # mask relevant bits

        if (status & 0b1001111) == 0:  # masking additional bit
            return NOV.STATE_NOT_READY_TO_SWITCH_ON
        elif (status & 0b1001111) == 0b1000000:  # masking additonal bit
            return NOV.STATE_SWITCH_ON_DISABLED
        elif status == 0b0100001:
            return NOV.STATE_READY_TO_SWITCH_ON
        elif status == 0b0100011:
            return NOV.STATE_SWITCHED_ON
        elif status == 0b0100111:
            return NOV.STATE_OPERATION_ENABLE
        elif status == 0b0000111:
            return NOV.STATE_QUICK_STOP
        elif (status & 0b1001111) == 0b0001111:  # masking additional bit
            return NOV.STATE_FAULT_REACTION
        elif (status & 0b1001111) == 0b0001000:  # masking additional bit
            return NOV.STATE_FAULT
        else:
            return None

    # -------------------------------------------------------------------------
    #
    #   STATE MACHINE
    #   The state machine within the Everest is best documented in
    #   the reference page for register 0x010
    #
    # -------------------------------------------------------------------------

    # -------------------------------------------------------------------------
    #   ENTER_STATE()
    # -------------------------------------------------------------------------
    def enter_state(self, state):
        """Enters a specific state in the servo controller.

        This method attempts to enter a specific state in the servo controller.
        It sets the configuration buffer with the state value and sends the
        call response to the servo controller.

        Parameters:
            state (int): The state to enter.

        Returns:
            None
        """
        self.set_config_buffer(NOV.CONTROL_WORD, NOV.CMD_WRITE, False, state)
        self.send_receive_call_response()
        if self.verbose:
            pyb.delay(50)
            self.report_state()

    # -------------------------------------------------------------------------
    #   STATE_TRANSITION()
    # -------------------------------------------------------------------------
    def state_transition(self, transition):
        """Performs a state transition in the servo controller.

        This method performs a state transition in the servo controller. It
        determines the state to transition to based on the transition value and
        calls the enter_state() method to enter the new state.

        NOTE: transition 3 can have a delay before status word reports it's
        complete

        Parameters:
            transition (int): The transition value.

        Returns:
            None
        """
        if (
                transition == 2
                or transition == 6
                or transition == 8
        ):
            state = NOV.STATE_READY_TO_SWITCH_ON
        elif (
                transition == 3
                or transition == 5
        ):
            state = NOV.STATE_SWITCHED_ON
        elif transition == 4:
            state = NOV.STATE_OPERATION_ENABLE
        elif (
                transition == 7
                or transition == 9
                or transition == 10
                or transition == 12
        ):
            state = NOV.STATE_SWITCH_ON_DISABLED
        elif transition == 11:
            state = NOV.STATE_QUICK_STOP
        elif transition == 15:
            state = NOV.STATE_FAULT_RESET
        else:
            print('INVALID TRANSITION')
            return

        self.enter_state(state)

    # -------------------------------------------------------------------------
    #   INIT_CRC_ARM()
    # -------------------------------------------------------------------------
    def init_crc_arm(self):
        """Initializes the CRC peripheral for the STM32H7 microcontroller.

        This method initializes the CRC peripheral for the STM32H7
        microcontroller. It sets the CRC polynomial and configuration registers
        to enable the CRC calculation unit.

        Parameters:
            None

        Returns:
            None
        """
        # -----------------------
        # Register address names:
        # -----------------------
        # stm.CRC:          CRC registers starting address
        # stm.CRC_CR:       configuration register offset
        # stm.CRC_DR:       data register offset
        # stm.CRC_IDR:      temp storage register (8 bits) offset
        # stm.CRC_INIT:     CRC calculation initial value register offset
        # stm.CRC_POL:      CRC polynomial
        #
        # stm.RCC:          Clock configuration starting address
        # stm.RCC_AHB4ENR:  offset to register with bitmask to enable AHB clocks
        #                   (Portenta_H7)
        # stm.RCC_AHB1ENR:  offset to register with bitmask to enable AHB clocks
        #                   (Pyboard)

        # ---------------------------------------------
        # The CRC_CR bitmask is constructed as follows:
        # ---------------------------------------------

        # CRC[0] - writing 0 resets calculation unit

        # CRC[1:2] - reserved

        # CRC[3:4] - sets polynomial size:
        # 32-bit polynomial:    0b00 << 3
        # 16-bit polynomial:    0b01 << 3
        #  8-bit polynomial:    0b10 << 3
        #  7-bit polynomial:    0b11 << 3

        # CRC[5:6] - bit reversal applied to input:
        # bit reversal disabled:        0b00 << 5
        # bit reversal by byte:         0b01 << 5
        # bit reversal by half word:    0b10 << 5
        # bit reversal by word:         0b11 << 5

        # CRC[7]: bit reversal applied to output:
        # bit reversal disabled:        0b0 << 7
        # bit reversal enabled:         0b1 << 7

        if sys.implementation._machine == 'Arduino Portenta H7 with STM32H747':
            # Enable the CRC real-time clock - STM_H7
            stm.mem32[stm.RCC + stm.RCC_AHB4ENR] = (
                    stm.mem32[stm.RCC + stm.RCC_AHB4ENR]
                    | (1 << 19)
            )

        else:
            # Enable the CRC real-time clock - STM_F7
            stm.mem32[stm.RCC + stm.RCC_AHB1ENR] = (
                    stm.mem32[stm.RCC + stm.RCC_AHB1ENR]
                    | (1 << 12)
            )

        # Reset and configure the CRC calculation peripheral (takes about 10 us)
        stm.mem32[stm.CRC + stm.CRC_CR] = (
            (0b01 << 3)
        )

        stm.mem32[stm.CRC + stm.CRC_POL] = 0x1021  # set the polynomial

    # -------------------------------------------------------------------------
    #   CRC_ARM()
    #   fast - about 7us
    # -------------------------------------------------------------------------
    @micropython.viper
    def crc_arm(self, data, size: int) -> int:
        """Calculates the CRC value using the STM32H7 CRC peripheral.

        This method calculates the CRC value using the STM32H7 CRC peripheral.
        It resets the CRC calculation unit, sets the initial value, and
        performs the calculation on data[0:size].

        Parameters:
            data (bytearray): The data buffer to calculate the CRC value.
            size (int): The size of the data buffer.

        Returns:
            int: The CRC value.
        """

        # reset the calculation
        ptr32(stm.CRC + stm.CRC_INIT)[0] = 0  # type: ignore

        # perform the calculation
        ptr_CRC_DR = ptr8(stm.CRC + stm.CRC_DR)  # type: ignore
        ptr_data = ptr8(data)  # type: ignore
        for i in range(size):
            ptr_CRC_DR[0] = ptr_data[i]

        return ptr32(stm.CRC + stm.CRC_DR)[0]  # type: ignore

    # -------------------------------------------------------------------------
    #   CRC_SOFTWARE()
    #   slow - about 160 us
    # -------------------------------------------------------------------------
    @micropython.native
    def crc_software(self, data, size=10, poly=0x1021, initial=0x0):
        """Calculates the CRC value using the software algorithm.

        This method calculates the CRC value using the software algorithm. It
        performs the calculation on data[0:size] using the polynomial and
        initial value provided.

        Parameters:
            data (bytearray): The data buffer to calculate the CRC value.
            size (int): The size of the data buffer.
            poly (int): The polynomial value.
            initial (int): The initial value.

        Returns:
            int: The CRC value.
        """

        '''
        CRC-16-CCITT (XModem) Algorithm
        data: bytearray of ints, MSB first

        algorithm from:
        https://en.wikipedia.org/wiki/Computation_of_cyclic_redundancy_checks
        (code fragment 4, as of 2022-01-24)

        online reference implementation for testing:
        https://www.lammertbies.nl/comm/info/crc-calculation

        NOTE: hardware CRC may be possible on STM32, but utilizing it requires
        configuring low-level registers. See:
        https://www.st.com/resource/en/application_note/dm00068118-using ...
        ... -the-crc-peripheral-in-the-stm32-family-stmicroelectronics.pdf
        '''

        crc = initial

        # for byte in data[0:size]:             # not ISR safe
        for i in range(size):
            byte = data[i]
            crc = crc ^ (byte << 8)
            for _ in range(8):
                if crc & (0x8000):  # 0x8000 = 1 << 15
                    crc = (crc << 1) ^ poly
                else:
                    crc <<= 1
                crc = crc & 0xFFFF

        return crc

    # -------------------------------------------------------------------------
    #   GET_REGISTER_INFO()
    # -------------------------------------------------------------------------
    def get_register_info(self, address):
        """Gets information about a specific register.

        This method gets information about a specific register. It reads the
        register information and returns the cyclic type, data type, size, and
        access type.

        Parameters:
            address (int): The address of the register to access.

        Returns:
            dict: A dictionary containing the register information.

        Example:
            {
                'cyclic_type': 1,
                'data_type': 3,
                'size': 4,
                'access_type': 1
            }
        """
        self.set_config_buffer(address, NOV.CMD_GET_INFO, False, 0)
        self.send_receive_call_response()

        response_cmd = (self.channel_config.header_mv[1] >> 1) & 0b111
        if response_cmd != NOV.CMD_ACK:
            arr = self.channel_config.config_data_mv
            print(
                f'GET_INFO 0x{address:03x}: ',
                f'response_cmd={response_cmd} (expected {NOV.CMD_ACK}) ',
                f'raw={bytes(arr[:4]).hex()}'
            )

        data = self.channel_config.int_from_config_data()

        size = data & 0xFF
        data_type = (data >> 8) & 0b111111
        cyclic_type = (data >> 14) & 0b11
        access_type = (data >> 16) & 0b111

        return {
            'cyclic_type': cyclic_type,
            'data_type': data_type,
            'size': size,
            'access_type': access_type
        }

    # -------------------------------------------------------------------------
    #   OPERATION_ENABLE_CYCLIC()
    # -------------------------------------------------------------------------
    @micropython.native
    def operation_enable_cyclic(self):
        """Enables cyclic operation for the servo controller.

        This method enables cyclic operation for the servo controller. It
        performs state transitions to switch the servo controller to the
        operation enable state.

        While servo is in cyclic mode,
            if state is SWITCH ON DISABLED, perform transition:
                SWITCH ON DISABLED -> READY TO SWITCH ON    (transition [2])

            then, with state at READY TO SWITCH ON, perform transitions:
                READY TO SWITCH ON -> SWITCHED ON           (transition [3])
                SWITCHED ON -> OPERATION ENABLE             (transition [4])

        Parameters:
            None

        Returns:
            int: The status word value.
        """
        header_buffer = self.channel_pico.header_mv

        #
        #   If we are in SWITCH ON DISABLED, perform transition #2 to READY TO
        #   SWITCH ON:
        #
        status = self.channel_poci.status_word & 0b1001111
        if status == 0b1000000:  # if state is SWITCH_ON_DISABLED
            header_buffer[0] = (NOV.CONTROL_WORD & 0xFF0) >> 4
            header_buffer[1] = (
                    ((NOV.CONTROL_WORD & 0xF) << 4)
                    | ((NOV.CMD_WRITE & 0b111) << 1)
            )
            self.channel_pico.int_to_config_data(393216)
            # (393216 = NOV.STATE_READY_TO_SWITCH_ON >> 32)

            self.send_receive_cyclic()

            self.channel_pico.set_idle()

            count = 0
            status = self.channel_poci.status_word & 0b1101111
            while (
                    not self.channel_poci.last_error
                    and (status ^ 0b100001)
                    and count < 2000
            ):
                self.send_receive_cyclic()
                status = self.channel_poci.status_word & 0b1101111
                count += 1

            if status ^ 0b100001:
                # print('Failed transition #2. count =', count)
                return status

        #
        #   Perform state transition #3 from READY TO SWITCH ON to SWITCHED ON
        #
        header_buffer[0] = (NOV.CONTROL_WORD & 0xFF0) >> 4
        header_buffer[1] = (
                ((NOV.CONTROL_WORD & 0xF) << 4)
                | ((NOV.CMD_WRITE & 0b111) << 1)
        )

        self.channel_pico.int_to_config_data(458752)
        # (458752 = NOV.STATE_SWITCHED_ON >> 32)

        self.send_receive_cyclic()

        self.channel_pico.set_idle()

        count = 0
        status = self.channel_poci.status_word & 0b1101111
        while (
                not self.channel_poci.last_error
                and (status ^ 0b100011)
                and count < 4000
        ):
            self.send_receive_cyclic()
            status = self.channel_poci.status_word & 0b1101111
            count += 1

        if status ^ 0b100011:
            # print('Failed transition #3. count =', count)
            return status  # failed to reach state SWITCHED ON

        #
        #   Perform state transition # 4 from SWITCHED ON to OPERATION ENABLE
        #
        header_buffer[0] = (NOV.CONTROL_WORD & 0xFF0) >> 4
        header_buffer[1] = (
                ((NOV.CONTROL_WORD & 0xF) << 4)
                | ((NOV.CMD_WRITE & 0b111) << 1)
        )

        self.channel_pico.int_to_config_data(983040)
        # (983040 = NOV.STATE_OPERATION_ENABLE >> 32)

        self.send_receive_cyclic()

        self.channel_pico.set_idle()

        count = 0
        status = self.channel_poci.status_word & 0b1101111
        while (
                not self.channel_poci.last_error
                and (status ^ 0b100111)
                and count < 2000
        ):
            self.send_receive_cyclic()
            status = self.channel_poci.status_word & 0b1101111
            count += 1

        # print('STATE 4')
        if status ^ 0b100111:
            # print('Failed transition #4. count =', count)
            return status  # failed to reach state OPERATION ENABLE
        else:
            return 0  # success

    # -------------------------------------------------------------------------
    #   OPERATION_DISABLE_CYCLIC()
    # -------------------------------------------------------------------------
    @micropython.native
    def operation_disable_cyclic(self):
        """Disables cyclic operation for the servo controller.

        This method disables cyclic operation for the servo controller. It
        performs state transitions to switch the servo controller to the
        switch on disabled state.

        While servo is in cyclic mode and state is OPERATION ENABLE,
        perform transitions:
            OPERATION ENABLE -> SWITCH ON DISABLED      (transition [7])

        Parameters:
            None

        Returns:
            int: The status word value.
        """
        header_buffer = self.channel_pico.header_mv

        #
        #   State transition #8:
        #
        header_buffer[0] = (NOV.CONTROL_WORD & 0xFF0) >> 4
        header_buffer[1] = (
                ((NOV.CONTROL_WORD & 0xF) << 4)
                | ((NOV.CMD_WRITE & 0b111) << 1)
        )
        self.channel_pico.int_to_config_data(393216)
        # (393216 = NOV.STATE_READY_TO_SWITCH_ON >> 32)

        self.send_receive_cyclic()

        self.channel_pico.set_idle()

        count = 0
        status = self.channel_poci.status_word & 0b1101111
        while (
                not self.channel_poci.last_error
                and (status ^ 0b100001)
                and count < 500
        ):
            self.send_receive_cyclic()
            status = self.channel_poci.status_word & 0b1101111
            count += 1

        # print('STATE 2')

        if status ^ 0b100001:
            # print('Failed transition #8. count =', count)
            return status  # failed to reach state READY TO SWITCH ON
        else:
            return 0  # success

    #   DICTIONARY FOR REVERSE LOOKUP OF CMD_*
    CMD_DICT = {
        NOV.CMD_GET_INFO: 'CMD_GET_INFO',
        NOV.CMD_READ: 'CMD_READ',
        NOV.CMD_WRITE: 'CMD_WRITE',
        NOV.CMD_ACK: 'CMD_ACK_READ',
        NOV.CMD_INFO_ERROR: 'CMD_INFO_ERROR',
        NOV.CMD_READ_ERROR: 'CMD_READ_ERROR',
        NOV.CMD_WRITE_ERROR: 'CMD_WRITE_ERROR',
        NOV.CMD_IDLE: 'CMD_IDLE'
    }

    # -------------------------------------------------------------------------
    #   PRINT_BUFFER()
    # -------------------------------------------------------------------------
    def print_buffer(self, channel=None):
        """Prints the buffer for a specific channel.

        This method prints the buffer for a specific channel. It reads the
        buffer values and prints the address, command, pending, data, word 1,
        word 2, word 3, word 4, and CRC values.

        Parameters:
            channel (int): The channel to print the buffer for.

            Returns:
                None
        """
        if not channel:
            if self.communication_mode == self._COMMUNICATION_MODE_CYCLIC:
                channel = self.channel_poci
            else:
                channel = self.channel_config
        elif channel == 1:
            channel = self.channel_pico
        elif channel == 2:
            channel = self.channel_poci
        else:
            print('Invalid channel passed to print_buffer()')
            return

        buffer = channel.buffer
        buffer_size = channel.CRC_data_len + 2

        address = (buffer[0] << 4) + (buffer[1] >> 4)
        command = (buffer[1] >> 1) & 0b111
        pending = buffer[1] & 0b1
        crc = (channel.CRC_sum_mv[0] << 8) | (channel.CRC_sum_mv[1])

        # config data as bytes
        word_4 = (buffer[2] << 8) + buffer[3]
        word_3 = (buffer[4] << 8) + buffer[5]
        word_2 = (buffer[6] << 8) + buffer[7]
        word_1 = (buffer[8] << 8) + buffer[9]

        # data as int
        data_int = (word_3 << 16) | word_4

        # data as float
        data_float = channel.float_from_config_data()

        # Print header
        print('address:\t0x{0:0{1}X}'.format(address, 3))
        try:
            print(
                'command:\t0b{0:0{1}b}\t{2}'.format(
                    command, 3, self.CMD_DICT[command]
                )
            )
        except:
            print('command:\t0b{0:0{1}b}'.format(command, 3))
        print('pending:\t' + str(pending))

        # Print config data
        print('data (hex):\t0x ', end='')
        for i in range(3, -1, -1):
            print('{0:0{1}X} '.format((data_int >> (i * 8)) & 0xFF, 2), end='')
        print('')
        print('data (bin):\t0b ', end='')
        for i in range(3, -1, -1):
            print('{0:0{1}b} '.format((data_int >> (i * 8)) & 0xFF, 8), end='')
        print('')
        print('data (int):\t{0}'.format(data_int))
        print('data (float):\t{0}'.format(data_float))
        print('word 1:\t\t0x{0:0{1}X}'.format(word_1, 4))
        print('word 2:\t\t0x{0:0{1}X}'.format(word_2, 4))
        print('word 3:\t\t0x{0:0{1}X}'.format(word_3, 4))
        print('word 4:\t\t0x{0:0{1}X}'.format(word_4, 4))

        # Print cyclic data
        if buffer_size > 12:
            print('--')
            n = 0
            for i in range(10, buffer_size - 2, 2):
                word = (buffer[i] << 8) | buffer[i + 1]
                print('word {2}:\t\t0x{0:0{1}X}'.format(word, 4, n))
                n = n + 1

        # Print CRC
        print('CRC:\t\t0x{0:0{1}X}'.format(crc, 4))
        print('CRC check:\t', self.crc16(buffer, buffer_size), sep='')

        print('')

    # -------------------------------------------------------------------------
    #   PRINT_RAW_BUFFER()
    # -------------------------------------------------------------------------
    def print_raw_buffer(self, print_buffer):
        """Prints the raw buffer values.

        This method prints the raw buffer values. It reads the buffer values
        and prints the hexadecimal values for each byte in the buffer.

        Parameters:
            print_buffer (bytearray): The buffer to print.

        Returns:
            None
        """
        for byte in print_buffer:
            print('x{0:0{1}X}'.format(byte, 2))


'''
╔════════════════════════════════════════════════════════════════════════════╗

 NOVANTA DATA FRAME STRUCTURE

╚════════════════════════════════════════════════════════════════════════════╝
┌───────────────────────────┐   ┌───────────────────────────┐
│HEADER                     │   │HEADER                     │
│(1 word)                   │   │(1 word)                   │
│-   -   -   -   -   -   -  │   │-   -   -   -   -   -   -  │
│address:            12 bits│   │address:            12 bits│
│command:             3 bits│   │command:             3 bits│
│pending frame:       1 bit │   │pending frame:       1 bit │
├───────────────────────────┤   ├───────────────────────────┤
│CONFIG DATA                │   │CONFIG DATA                │
│(4 words)                  │   │(4 words)                  │
├───────────────────────────┤   ├───────────────────────────┤
│CRC                        │   │CYCLIC DATA                │
│(1 word)                   │   │(0-32 words)               │
└───────────────────────────┘   ├───────────────────────────┤
 data frame in configuration    │CRC                        │
 state                          │(1 word)                   │
                                └───────────────────────────┘
                                 data frame in cyclic state
'''


# -----------------------------------------------------------------------------
#
#   CLASS CHANNEL()
#
# -----------------------------------------------------------------------------
class Channel:
    """Communication channel used in communicating with Everest servo.

    This class defines a communication channel for the Everest servo controller.
    It contains methods to set and get values from the channel buffer.

    Parameters:
        servo_instance: Instance of the Everest servo controller.
        channel_type: Type of channel (CONFIG, PICO, POCI).
        cyclic_size: Size of the cyclic data buffer.
        buffer: Entire buffer [0 : buffer_size].
        header_mv: Header bytes [0 : 2].
        config_data_mv: Configuration data portion [2 : 10].
        cyclic_data_mv: Cyclic data portion [10 : 10 + cyclic_size].
        CRC_data_mv: Data used to calculate CRC [0 : 10 + cyclic_size].
        CRC_sum_mv: CRC checksum [10 + cyclic_size : 12 + cyclic_size].
        CRC_data_len: Length of the CRC data.

    Attributes:
        servo (object): Instance of the Everest servo controller.
        channel_type (int): Type of channel (CONFIG, PICO, POCI).
        cyclic_size (int): Size of the cyclic data buffer.
        float_holder (upy_float): Holder for float data.
        int16_holder (array): Holder for int16 data.
        int16_adr (int): Address of int16_holder.
        uint16_holder (array): Holder for uint16 data.
        uint16_adr (int): Address of uint16_holder.
        int32_holder (array): Holder for int32 data.
        int32_adr (int): Address of int32_holder.
        uint32_holder (array): Holder for uint32 data.

    Note:
        Trailing zeros may exist at the end of .buffer to match the size of the
        other channel.
    """
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #   CLASS VARIABLES
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    CONFIG = 0
    PICO = 1
    POCI = 2

    # -------------------------------------------------------------------------
    #   __INIT__()
    # -------------------------------------------------------------------------
    def __init__(
            self,
            servo_instance,
            # cyclic_size=0,
            # cyclic_registers=None,
            channel_type=None
    ):

        self.servo = servo_instance

        if not channel_type:
            self.channel_type = Channel.CONFIG
            self.cyclic_size = 0
        else:
            self.channel_type = channel_type
            self.cyclic_size = self.servo.cyclic_size

        # These holders are used by bound property methods to get and set
        # values to and from the cyclic data. '''
        self.float_holder = upy_float.upyFloat(0.0)

        self.int16_holder = array.array('h', [0])
        self.int16_adr = addressof(self.int16_holder)

        self.uint16_holder = array.array('H', [0])
        self.uint16_adr = addressof(self.uint16_holder)

        self.int32_holder = array.array('i', [0])
        self.int32_adr = addressof(self.int32_holder)

        self.uint32_holder = array.array('I', [0])
        self.uint32_adr = addressof(self.uint32_holder)

        self.buffer_size = 12 + self.cyclic_size

        self.buffer = bytearray(self.buffer_size)
        self.buffer_mv = memoryview(self.buffer)

        # establish memoryviews to access areas of entire buffer
        self.header_mv = memoryview(self.buffer_mv[0:2])
        self.config_data_mv = memoryview(self.buffer_mv[2:10])
        if self.cyclic_size:
            self.cyclic_data_mv = memoryview(
                self.buffer_mv[10:10 + self.cyclic_size]
            )
        self.CRC_data_mv = memoryview(self.buffer_mv[0:self.cyclic_size + 10])
        self.CRC_sum_mv = memoryview(
            self.buffer_mv[self.cyclic_size + 10: self.cyclic_size + 12]
        )
        self.CRC_data_len = len(self.CRC_data_mv)

        if self.channel_type != Channel.CONFIG:

            if self.channel_type == Channel.PICO:
                self.cyclic_dict = dict(self.servo.pico_cyclic_dict)
            elif self.channel_type == Channel.POCI:
                self.cyclic_dict = dict(self.servo.poci_cyclic_dict)

            self._generate_property_methods()

    # # -------------------------------------------------------------------------
    # #   GENERATE_PROPERTY_METHODS()
    # # -------------------------------------------------------------------------
    # # def _generate_property_methods(self, name, data_type, offset):
    def _generate_property_methods(self):
        """Generates property methods for the channel.

        This method generates property methods for the channel. It creates
        getter and setter methods based on the data type of the register.

        Parameters:
            name (str): Name of the property.
            data_type (int): Data type of the register.
            offset (int): Byte offset in the cyclic data buffer

        Returns:
            None
        """

        offset = 0
        register_counter = 0

        if self.channel_type == Channel.PICO:
            base_address = 0x651
        elif self.channel_type == Channel.POCI:
            base_address = 0x661

        for register, params in self.cyclic_dict.items():

            name = params['name']
            data_type = params['data_type']
            size = params['size']

            #
            #   INT16
            #
            if data_type == NOV.TYPE_INT16:
                def getter(self, offset=offset):

                    arr_in = self.cyclic_data_mv
                    arr_out = self.int16_adr

                    stm.mem8[arr_out] = arr_in[offset + 1]
                    stm.mem8[arr_out + 1] = arr_in[offset]

                    return self.int16_holder[0]

                def setter(self, value, offset=offset):

                    arr_in = self.int16_adr
                    arr_out = self.cyclic_data_mv

                    self.int16_holder[0] = value

                    arr_out[offset] = stm.mem8[arr_in + 1]
                    arr_out[offset + 1] = stm.mem8[arr_in]

            #
            #   UINT16
            #
            elif data_type == NOV.TYPE_UINT16:
                def getter(self, offset=offset):

                    arr_in = self.cyclic_data_mv
                    arr_out = self.uint16_adr

                    stm.mem8[arr_out] = arr_in[offset + 1]
                    stm.mem8[arr_out + 1] = arr_in[offset]

                    return self.uint16_holder[0]

                def setter(self, value, offset=offset):

                    arr_in = self.uint16_adr
                    arr_out = self.cyclic_data_mv

                    self.uint16_holder[0] = value

                    arr_out[offset] = stm.mem8[arr_in + 1]
                    arr_out[offset + 1] = stm.mem8[arr_in]

            #
            #   INT32
            #
            elif data_type == NOV.TYPE_INT32:

                def getter(self, offset=offset):

                    arr_in = self.cyclic_data_mv
                    arr_out = self.int32_adr

                    stm.mem8[arr_out] = arr_in[offset + 1]
                    stm.mem8[arr_out + 1] = arr_in[offset]
                    stm.mem8[arr_out + 2] = arr_in[offset + 3]
                    stm.mem8[arr_out + 3] = arr_in[offset + 2]

                    return self.int32_holder[0]

                def setter(self, value, offset=offset):

                    arr_in = self.int32_adr
                    arr_out = self.cyclic_data_mv

                    self.int32_holder[0] = value

                    arr_out[offset] = stm.mem8[arr_in + 1]
                    arr_out[offset + 1] = stm.mem8[arr_in]
                    arr_out[offset + 2] = stm.mem8[arr_in + 3]
                    arr_out[offset + 3] = stm.mem8[arr_in + 2]

            #
            #   UINT32
            #
            elif data_type == NOV.TYPE_UINT32:
                def getter(self, offset=offset):

                    arr_in = self.cyclic_data_mv
                    arr_out = self.uint32_adr

                    stm.mem8[arr_out] = arr_in[offset + 1]
                    stm.mem8[arr_out + 1] = arr_in[offset]
                    stm.mem8[arr_out + 2] = arr_in[offset + 3]
                    stm.mem8[arr_out + 3] = arr_in[offset + 2]

                    return self.uint32_holder[0]

                def setter(self, value, offset=offset):

                    arr_in = self.uint32_adr
                    arr_out = self.cyclic_data_mv

                    self.uint32_holder[0] = value

                    arr_out[offset] = stm.mem8[arr_in + 1]
                    arr_out[offset + 1] = stm.mem8[arr_in]
                    arr_out[offset + 2] = stm.mem8[arr_in + 3]
                    arr_out[offset + 3] = stm.mem8[arr_in + 2]

            #
            #   FLOAT
            #
            elif data_type == NOV.TYPE_FLOAT:
                def getter(self, offset=offset):

                    arr_in = self.cyclic_data_mv
                    arr_out = self.float_holder.adr

                    stm.mem8[arr_out] = arr_in[offset + 1]
                    stm.mem8[arr_out + 1] = arr_in[offset]
                    stm.mem8[arr_out + 2] = arr_in[offset + 3]
                    stm.mem8[arr_out + 3] = arr_in[offset + 2]

                    return self.float_holder.val

                def setter(self, value, offset=offset):

                    arr_in = self.float_holder.adr
                    arr_out = self.cyclic_data_mv

                    self.float_holder.val = value

                    arr_out[offset] = stm.mem8[arr_in + 1]
                    arr_out[offset + 1] = stm.mem8[arr_in]
                    arr_out[offset + 2] = stm.mem8[arr_in + 3]
                    arr_out[offset + 3] = stm.mem8[arr_in + 2]

            else:
                raise ValueError("Unsupported data type")

            setattr(self.__class__, name, property(getter, setter))

            print(
                ' | '.join([
                    f'[register_counter: {register_counter:02d}]',
                    f'[offset: {offset:02d}]',
                    f'[size: {size:02d}]',
                    f'[data_type: {data_type: 02d}]',
                    f'"{name}"'
                ])
            )

            offset += size

            # Add register to servo's cyclic register list.
            self.servo.set_register(
                base_address + register_counter,
                (size << 16) | register,
                verbose=False
            )

            register_counter += 1

        # Set the number of cyclic registers in each transmission.
        self.servo.set_register(
            base_address - 1,
            register_counter,
            verbose=False
        )

    # -------------------------------------------------------------------------
    #   SET_IDLE()
    # -------------------------------------------------------------------------
    @micropython.native
    def set_idle(self) -> None:
        """Sets the header to idle state.

        This method sets the header to idle state, which avoids the POCI
        channel from raising an error for an unknown command.

        Parameters:
            None

        Returns:
            None
        """
        self.header_mv[0] = 0
        self.header_mv[1] = NOV.CMD_IDLE << 1
        self.config_data_mv[0] = 0
        self.config_data_mv[1] = 0
        self.config_data_mv[2] = 0
        self.config_data_mv[3] = 0
        self.config_data_mv[4] = 0
        self.config_data_mv[5] = 0
        self.config_data_mv[6] = 0
        self.config_data_mv[7] = 0

    # -------------------------------------------------------------------------
    #   SET_CRC()
    # -------------------------------------------------------------------------
    @micropython.native
    def set_crc(self, value: int) -> None:
        """Sets the CRC value in the buffer.

        This method sets the CRC value in the buffer. It writes the CRC value
        to the buffer.

        Parameters:
            value (int): The CRC value to set.

        Returns:
            None
        """
        self.CRC_sum_mv[0] = (value >> 8) & 0xFF
        self.CRC_sum_mv[1] = value & 0xFF

    # -------------------------------------------------------------------------
    #   FLOAT_TO_CONFIG_DATA()
    # -------------------------------------------------------------------------
    @micropython.native
    def float_to_config_data(self, input: float) -> None:
        """Packs a float into the configuration data buffer.

        This method takes a float as input and packs a 4-byte buffer in the
        ordering used by the data portion of a config_buffer.

        Parameters:
            input (float): The float value to pack.

        Returns:
            None
        """

        self.float_holder.val = input
        float_array = self.float_holder.raw_bytearray

        self.config_data_mv[0] = float_array[1]  # word 0 MSB
        self.config_data_mv[1] = float_array[0]
        self.config_data_mv[2] = float_array[3]
        self.config_data_mv[3] = float_array[2]  # word 1 LSB

    # -------------------------------------------------------------------------
    #   FLOAT_FROM_CONFIG_DATA()
    # -------------------------------------------------------------------------
    def float_from_config_data(self):
        """Unpacks a float from the configuration data buffer.

        This method reads bytes from the configuration data buffer and returns
        a float.

        Parameters:
            None

        Returns:
            float: The float value.
        """
        adr = self.float_holder.adr
        arr = self.config_data_mv

        stm.mem8[adr] = arr[1]
        stm.mem8[adr + 1] = arr[0]
        stm.mem8[adr + 2] = arr[3]
        stm.mem8[adr + 3] = arr[2]

        return self.float_holder.val

    # -------------------------------------------------------------------------
    #   INT_TO_CONFIG_DATA()
    # -------------------------------------------------------------------------
    @micropython.native
    def int_to_config_data(self, val: int) -> None:
        '''takes int and writes to config data buffer'''
        arr = self.config_data_mv
        arr[0] = (val >> 24) & 0x7F
        arr[1] = (val >> 16) & 0xFF
        arr[2] = (val >> 8) & 0xFF
        arr[3] = val & 0xFF

    # -------------------------------------------------------------------------
    #   INT_FROM_CONFIG_DATA()
    # -------------------------------------------------------------------------
    def int_from_config_data(self):
        """Reads bytes from the configuration data buffer and returns an int.

        This method reads bytes from the configuration data buffer and returns
        an int.

        Parameters:
            None

        Returns:
            int: The int value.
        """
        arr = self.config_data_mv
        return (
                (arr[2] & 0x7F) << 24  # 31-bit ints
                | arr[3] << 16
                | arr[0] << 8
                | arr[1]
        )
