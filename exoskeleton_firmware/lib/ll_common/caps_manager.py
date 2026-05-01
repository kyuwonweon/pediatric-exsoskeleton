import array                            # type: ignore
import micropython                      # type: ignore
import gc
from uctypes import addressof           # type: ignore

print('IMPORTING CAPS MANAGER')

class CapsManager:
    """
    Manages CAN input and output feeds to interface with CAPS.
    This class acts as a support module tied to the HybridLeg instance
    and creates CAPS objects as attributes of the parent instance.
    """

    def __init__(self, parent):
        self.parent = parent

        # ====================================================================
        # PDCP NIDs
        # ====================================================================

        # --------------------------------------------------------------------
        # CAPS CAN OUTPUT DEVICES (COD)
        # --------------------------------------------------------------------
        # KNEE ACTUATOR
        # Control NID
        self.parent.CAPS1_ID = micropython.const(0x018)
        # Data NID : K, B, E, Flex/Extend
        self.parent.CAPS2_ID = micropython.const(0x019)
        # ANKLE ACTUATOR
        # Control NID
        self.parent.CAPS5_ID = micropython.const(0x0C8)
        # Data NID : K, B, E, Flex/Extend
        self.parent.CAPS6_ID = micropython.const(0x0C9)
        # KNEE DYNAMIC BRAKE
        # Control NID
        self.parent.CAPS7_ID = micropython.const(0x0D4)
        # Data NID : Brake Set Point, Brake Angle, Brake Velocity
        self.parent.CAPS8_ID = micropython.const(0x0D5)

        # --------------------------------------------------------------------
        # CAPS CAN INPUT DEVICES (CID)
        # --------------------------------------------------------------------
        # KNEE OUTPUT PRIMARY - Control and Data NID
        self.parent.CAPS9_ID = micropython.const(0x01E)
        self.parent.CAPS9_DATA_ID = micropython.const(0x01F) #can_out_mes1
        # KNEE OUTPUT SECONDARY - Control and Data NID
        self.parent.CAPS14_ID = micropython.const(0x020)
        self.parent.CAPS14_DATA_ID = micropython.const(0x021) #can_out_mes2
        # CVT OUTPUT PRIMARY - Control and Data NID
        self.parent.CAPS15_ID = micropython.const(0x02A)
        self.parent.CAPS15_DATA_ID = micropython.const(0x02B) #can_out_mes3
        # ANKLE OUTPUT PRIMARY - Control and Data NID
        self.parent.CAPS13_ID = micropython.const(0x0CE)
        self.parent.CAPS13_DATA_ID = micropython.const(0x0CF) #can_out_mes5
        # ANKLE OUTPUT SECONDARY - Control and Data NID
        # Currently unused / Extra channel
        self.parent.CAPS16_ID = micropython.const(0x0D0)
        self.parent.CAPS16_DATA_ID = micropython.const(0x0D1) #can_out_mes6
        # IMU - Control and Data NID
        self.parent.IMU_ID = micropython.const(0x02C)
        self.parent.IMU_DATA1_ID = micropython.const(0x02D) #can_out_mes_imu1
        self.parent.IMU_DATA2_ID = micropython.const(0x02E) #can_out_mes_imu2
        # LOAD CELL - Control and Data NID
        self.parent.LOAD_CELL1_ID = micropython.const(0x040)
        self.parent.LOAD_CELL1_DATA1_ID = micropython.const(0x041)
        self.parent.LOAD_CELL1_DATA2_ID = micropython.const(0x042)
        # FORCE/TORQUE SENSOR and ERROR FLAG - NID
        # Retained for potential future CAN output from on-board FTS
        self.parent.FTS_AND_ERROR_ID = micropython.const(0x0E8)

        # --------------------------------------------------------------------
        # Relevant CAPS NIDs not found in CID or COD
        # --------------------------------------------------------------------
        # BUZZER DATA
        self.parent.CAPS10_ID = micropython.const(0x0C0)
        # CAPS COMMUNICATION CHECK
        self.parent.CAPS11_ID = micropython.const(0x0F9)
        # CAPS COMMUNICATION CHECK
        self.parent.CAPS12_ID = micropython.const(0x0FA)

    def initialize_caps_handlers(self):
        """Initialize all CAPS PDCP NID handlers"""

        print('Initializing CAPS Input Channels')
        # Initialize all incoming CAPS NID handlers
        self.set_input_control_nids()
        self.set_input_main_joint_control()
        self.set_input_braking_control()
        self.set_input_loadcell_stream()

        # Create dictionary mapping of input CAN NIDs
        self.set_input_caps_mapping_dict()

        print('Initializing CAPS Output Channels')
        # Initialize all outgoing CAPS NID handlers
        self.set_output_imu_streaming()
        self.set_output_knee_joint1_streaming()
        self.set_output_knee_joint2_streaming()
        self.set_output_polycentric_ankle_streaming()
        self.set_output_fts_streaming()

        print('Finished Initializing CAPS Handlers')

    def set_input_control_nids(self):
        """Stream in non-data NIDs that provide Start/Stop commands,
        communication status information, and custom commands from CAPS."""
        # Control NIDs from CAPS CAN OUTPUT DEVICES (COD)
        self.parent.CAPS1 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
            0x100 + self.parent.CAPS1_ID, None, None, None, None, is_control_nid=True
        )
        self.parent.CAPS5 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
            0x100 + self.parent.CAPS5_ID, None, None, None, None, is_control_nid=True
        )
        self.parent.CAPS7 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
            0x100 + self.parent.CAPS7_ID, None, None, None, None, is_control_nid=True
        )
        # Primary Control NIDs from CAPS CAN INPUT DEVICES (CID)
        self.parent.CAPS9 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
            0x100 + self.parent.CAPS9_ID, None, None, None, None, is_control_nid=True
        )
        self.parent.CAPS13 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
            0x100 + self.parent.CAPS13_ID, None, None, None, None, is_control_nid=True
        )
        # Other relevant Control NIDs from CAPS
        self.parent.CAPS10 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
            0x100 + self.parent.CAPS10_ID, None, None, None, None, is_control_nid=True
        )
        self.parent.CAPS11 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
            0x400 + self.parent.CAPS11_ID, None, None, None, None, is_control_nid=True
        )
        self.parent.CAPS12 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
            0x400 + self.parent.CAPS12_ID, None, None, None, None, is_control_nid=True
        )
        # FTS control NID (only present on Fox Knee with FTS installed)
        if hasattr(self.parent, 'fts_nid_control'):
            self.parent.FTS_CONTROL = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
                0x100 + self.parent.fts_nid_control, None, None, None, None, is_control_nid=True
            )

    def set_input_main_joint_control(self):
        """Stream in knee and ankle impedance parameter messages from CAPs
        and place the parameters into their appropriate variables"""
        if self.parent.components.Everest.is_enabled():
            self.parent.CAPS2 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
                    0x300 + self.parent.CAPS2_ID,
                    12,
                    array.array('f', [0.5, 41.12, 0.5, 411.2, 0.5, 11.35, 0.5, 8.224]),
                    array.array('i', [
                            addressof(self.parent.everest_pb.controller_parameters) + (4 * 12),
                            addressof(self.parent.everest_pb.controller_parameters) + 4,
                            addressof(self.parent.everest_pb.controller_parameters) + (10 * 4),
                            addressof(self.parent.everest_pb.controller_parameters) + (19 * 4)
                    ]),
                    self.parent.can_send
                )
            self.parent.CAPS6 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
                    0x300 + self.parent.CAPS6_ID,
                    12,
                    array.array('f', [0.5, 41.12, 0.5, 411.2, 0.5, 11.35, 0.5, 8.224]),
                    array.array('i', [
                            addressof(self.parent.everest_pb.controller_parameters) + (4 * 12),
                            addressof(self.parent.everest_pb.controller_parameters) + 4,
                            addressof(self.parent.PARAM_DICT['ANKLE_EQ_PLACEHOLDER']),
                            addressof(self.parent.everest_pb.controller_parameters) + (19 * 4)
                    ]),
                    self.parent.can_send
                )

    def set_input_braking_control(self):
        """Stream in control messages from CAPs for braking parameters
        and place them into their appropriate variables"""
        if self.parent.components.BrakingCircuit.is_enabled():
            self.parent.CAPS8 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
                0x300 + self.parent.CAPS8_ID,
                16,
                array.array('f', [0.5, 327.667, 0.5, 180.67, 0.5, 327.67]),
                array.array('i', [
                    addressof(self.parent.PARAM_DICT['BRAKE_PERCENTAGE']),
                    addressof(self.parent.PARAM_DICT['BRAKE_ANGLE']),
                    addressof(self.parent.PARAM_DICT['BRAKE_RATE'])
                ]),
                self.parent.can_send
            )
        else:
            self.parent.CAPS8 = None

    def set_input_loadcell_stream(self):
        """Stream in loadcell data from loadcell board and
        place them into their appropriate variables"""
        if self.parent.device_parameters.PRIMARY_CONTROLLER:
            self.parent.LOAD_CELL1_DATA1 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
                0x400 + self.parent.LOAD_CELL1_DATA1_ID,
                16,
                array.array('f', [0.5, 6553.6, 0.5, 6553.6, 0.5, 6553.6]),
                array.array('i', [
                    addressof(self.parent.sm_channels_current) + (4 * 1),
                    addressof(self.parent.sm_channels_current) + (4 * 2),
                    addressof(self.parent.sm_channels_current) + (4 * 3)
                ]),
                self.parent.can_send
            )

            self.parent.LOAD_CELL1_DATA2 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
                0x400 + self.parent.LOAD_CELL1_DATA2_ID,
                16,
                array.array('f', [0.5, 6553.6, 0.5, 6553.6, 0.5, 6553.6]),
                array.array('i', [
                    addressof(self.parent.sm_channels_current) + (4 * 4),
                    addressof(self.parent.sm_channels_current) + (4 * 5),
                    addressof(self.parent.sm_channels_current) + (4 * 6)
                ]),
                self.parent.can_send
            )
        else:
            self.parent.LOAD_CELL1_DATA1 = None
            self.parent.LOAD_CELL1_DATA2 = None

    def set_input_caps_mapping_dict(self):
        """Create dictionary mapping of input CAN IDs for
        can_class.py to determine the current active_channel"""
        self.parent.CAN_INPUT_DICT = {
            self.parent.CAPS1_ID: self.parent.CAPS1,
            self.parent.CAPS2_ID: self.parent.CAPS2,
            self.parent.CAPS5_ID: self.parent.CAPS5,
            self.parent.CAPS6_ID: self.parent.CAPS6,
            self.parent.CAPS7_ID: self.parent.CAPS7,
            self.parent.CAPS8_ID: self.parent.CAPS8,
            self.parent.CAPS9_ID: self.parent.CAPS9,
            self.parent.CAPS10_ID: self.parent.CAPS10,
            self.parent.CAPS11_ID: self.parent.CAPS11,
            self.parent.CAPS12_ID: self.parent.CAPS12,
            self.parent.CAPS13_ID: self.parent.CAPS13,
            self.parent.LOAD_CELL1_DATA1_ID: self.parent.LOAD_CELL1_DATA1,
            self.parent.LOAD_CELL1_DATA2_ID: self.parent.LOAD_CELL1_DATA2
        }
        if hasattr(self.parent, 'fts_nid_control'):
            self.parent.CAN_INPUT_DICT[self.parent.fts_nid_control] = self.parent.FTS_CONTROL
        gc.collect()

    def set_output_imu_streaming(self):
        """ Set up output streaming of IMU data.
            Currently set in CAPS as:
                can_out_mes_imu1: Channels 16 to 19
                can_out_mes_imu2: Channels 20 to 23
        """
        if self.parent.components.IMU.is_enabled():
            self.parent.can_out_mes_imu1 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
                0x400 + self.parent.IMU_DATA1_ID,
                12,
                array.array('f', [0.5, 182.564, 0.5, 182.564, 0.5, 182.564, 0.5, 182.564]),
                array.array('i', [
                    addressof(self.parent.imu_data),
                    addressof(self.parent.imu_data) + 4,
                    addressof(self.parent.imu_data) + 8,
                    addressof(self.parent.imu_data) + 12
                ]),
                self.parent.can_send
            )

            self.parent.can_out_mes_imu2 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
                0x400 + self.parent.IMU_DATA2_ID,
                12,
                array.array('f', [0.5, 182.564, 0.5, 182.564, 0.5, 182.564, 0.5, 182.564]),
                array.array('i', [
                    addressof(self.parent.imu_data) + 16,
                    addressof(self.parent.imu_data) + 20,
                    addressof(self.parent.imu_data) + 24,
                    addressof(self.parent.imu_data) + 28
                ]),
                self.parent.can_send
            )

    def set_output_knee_joint1_streaming(self):
        """ Set up output streaming of knee signals.
            Currently set in CAPS as:
                can_out_mes1: Channels 8 to 11
                can_out_mes2: Channels 24 to 27
        """
        if (self.parent.components.Everest.is_enabled()
            and (
                self.parent.device_parameters.device == self.parent.config.HYBRID_KNEE
                or self.parent.device_parameters.device == self.parent.config.FOX_KNEE
            )
        ):
            self.parent.can_out_mes1 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
                0x400 + self.parent.CAPS9_DATA_ID,
                12,
                array.array('f', [0.5, 182.564, 0.5, 13.0879, 0.5, 327.67, 0.5, 327.67]),
                array.array('i', [
                    addressof(self.parent.PARAM_DICT['JOINT_ANGLE']),
                    addressof(self.parent.PARAM_DICT['JOINT_SPEED_ARRAY']),
                    addressof(self.parent.PARAM_DICT['LARGE_MOTOR_REFERENCE_CURRENT']),
                    addressof(self.parent.PARAM_DICT['LARGE_MOTOR_CURRENT_COMMAND'])
                ]),
                self.parent.can_send
            )

            self.parent.can_out_mes2 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
                0x400 + self.parent.CAPS14_DATA_ID,
                12,
                array.array('f', [0.5, 655.35, 0.5, 100.0, 0.5, 6553.6, 0.5, 1.0]),
                array.array('i', [
                    addressof(self.parent.PARAM_DICT['BATTERY_VOLTS']),
                    addressof(self.parent.PARAM_DICT['ACTIVE_STATE']),
                    addressof(self.parent.PARAM_DICT['SUBJECT_BUTTON_STATE']),
                    addressof(self.parent.PARAM_DICT['CAPS_COMMUNICATION_VALUE'])
                ]),
                self.parent.can_send
            )

    def set_output_knee_joint2_streaming(self):
        """ Set up output streaming of secondary knee signals.
            Currently set in CAPS as:
                can_out_mes3: Channels 28 to 31.
        """
        if self.parent.components.BrakingCircuit.is_enabled():
            if self.parent.device_parameters.device == self.parent.config.HYBRID_KNEE:
                self.parent.can_out_mes3 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
                    0x400 + self.parent.CAPS15_DATA_ID,
                    12,
                    array.array('f', [0.5, 65.5, 0.5, 65.5, 0.5, 65.5, 0.5, 65.5]),
                    array.array('i', [
                        addressof(self.parent.PARAM_DICT['BRAKE_PERCENTAGE_CAPS']),
                        addressof(self.parent.PARAM_DICT['BRAKE_PERCENTAGE_CAPS']),
                        addressof(self.parent.PARAM_DICT['BRAKE_PERCENTAGE_CAPS']),
                        addressof(self.parent.PARAM_DICT['BRAKE_PERCENTAGE_CAPS'])
                    ]),
                    self.parent.can_send
                )
            elif self.parent.device_parameters.device == self.parent.config.FOX_KNEE:
                self.parent.can_out_mes3 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
                    0x400 + self.parent.CAPS15_DATA_ID,
                    12,
                    array.array('f', [0.5, 65.535, 0.5, 65.5, 0.5, 65.5, 0.5, 65.5]),
                    array.array('i', [
                        addressof(self.parent.PARAM_DICT['MOTOR_TEMP']) + (0),
                        addressof(self.parent.PARAM_DICT['BRAKE_PERCENTAGE_CAPS']),
                        addressof(self.parent.PARAM_DICT['BRAKE_PERCENTAGE_CAPS']),
                        addressof(self.parent.PARAM_DICT['BRAKE_PERCENTAGE_CAPS'])
                    ]),
                    self.parent.can_send
                )

    def set_output_polycentric_ankle_streaming(self):
        """ Set up output streaming of ankle signals.
            Currently set in CAPS as:
                can_out_mes5: Channels 12 to 15
        """
        if self.parent.components.Everest.is_enabled():
            if (
                self.parent.device_parameters.device == self.parent.config.POLYCENTRIC_ANKLE
                or self.parent.device_parameters.device == self.parent.config.POLYCENTRIC_ANKLE_ONLY
            ):
                self.parent.can_out_mes5 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
                    0x400 + self.parent.CAPS13_DATA_ID,
                    12,
                    array.array('f', [0.5, 182.564, 0.5, 13.0879, 0.5, 327.67, 0.5, 327.67]),
                    array.array('i', [
                        addressof(self.parent.PARAM_DICT['JOINT_ANGLE']),
                        addressof(self.parent.PARAM_DICT['JOINT_SPEED_ARRAY']),
                        addressof(self.parent.PARAM_DICT['LARGE_MOTOR_REFERENCE_CURRENT']),
                        addressof(self.parent.PARAM_DICT['LARGE_MOTOR_CURRENT_COMMAND'])
                    ]),
                    self.parent.can_send
                )

                # Unused can output message
                # self.parent.can_out_mes6 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
                #     0x400 + self.parent.CAPS16_DATA_ID,
                #     12,
                #     array.array('f', [0.5, 6553.6, 0.5, 6553.6, 0.5, 6553.6, 0.5, 6553.6]),
                #     array.array('i', [0, 0, 0, 0]),
                #     self.parent.can_send
                # )

    def set_output_fts_streaming(self):
        """Set up output streaming of FTS data over CAN as a drop-in
        replacement for the external load cell PDCP structure.

        Data NID 1 (force channels): [zero, zero, Fz] — Fz in ch3 position
        Data NID 2 (moment channels): [zero, My, zero] — My in ch5 position

        Gains encode physical values so the standard CAPS decode formula
        (raw / 6553.5 - 5.0) * LC_SCALE recovers calibrated N and Nm.
        Force gain = 6553.5 / 500 = 13.107, moment gain = 6553.5 / 40 = 163.8375.
        """
        if hasattr(self.parent, 'fts_nid_data1'):
            self.parent._fts_zero = array.array('f', [0.0])
            zero_addr = addressof(self.parent._fts_zero)

            print('Setting up FTS CAN streaming')
            self.parent.can_out_fts_data1 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
                0x400 + self.parent.fts_nid_data1,
                16,
                array.array('f', [0.5, 13.107, 0.5, 13.107, 0.5, 13.107]),
                array.array('i', [
                    zero_addr,
                    zero_addr,
                    addressof(self.parent.PARAM_DICT['FTS_FZ'])
                ]),
                self.parent.can_send
            )

            self.parent.can_out_fts_data2 = self.parent.CAN_PDCP_Streaming_Message.CanPdcpNidHandler(
                0x400 + self.parent.fts_nid_data2,
                16,
                array.array('f', [0.5, 163.8375, 0.5, 163.8375, 0.5, 163.8375]),
                array.array('i', [
                    zero_addr,
                    addressof(self.parent.PARAM_DICT['FTS_MY']),
                    zero_addr
                ]),
                self.parent.can_send
            )

    def clear_all_caps_data(self):
        """Function to clear all caps data"""
        self.parent.CAPS1.zero()
        self.parent.CAPS2.zero_and_clear()
        self.parent.CAPS5.zero()
        self.parent.CAPS6.zero_and_clear()
        self.parent.CAPS7.zero()
        # self.parent.CAPS8.zero_and_clear()
        self.parent.CAPS9.zero()
        # self.parent.CAPS10.zero()
        self.parent.CAPS13.zero()
        if self.parent.device_parameters.PRIMARY_CONTROLLER:
            self.parent.LOAD_CELL1_DATA1.zero()
            self.parent.LOAD_CELL1_DATA2.zero()
        if hasattr(self.parent, 'can_out_fts_data1'):
            self.parent.can_out_fts_data1.zero()
            self.parent.can_out_fts_data2.zero()
