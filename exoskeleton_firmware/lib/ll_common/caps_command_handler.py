import pyb                              # type: ignore

print('IMPORTING CAPS COMMAND HANDLER')

class CapsCommandHandler:
    """
    Handles the logic for CAN msg generator commands that come in from CAPS
    """

    def __init__(self, parent):
        self.parent = parent

    def rehome_encoder(self, _):

        self.parent.report_status('Rehoming joint absolute encoder')

        # Buzz to notify that we are about to write to everest
        self.parent.led_buzzer.play_buzzer(400, 2000, 90, "ALL")
        # Call Novanta function to set Joint offset
        self.parent.servo.set_joint_offset()

        if (
            self.parent.PARAM_DICT['MAIN_JOINT_ENCODER_VALUE'][0] < 
            self.parent.CONST_DICT['MAIN_JOINT_HOMING_WINDOW']
            and self.parent.PARAM_DICT['MAIN_JOINT_ENCODER_VALUE'][0] > 
            -self.parent.CONST_DICT['MAIN_JOINT_HOMING_WINDOW']
        ):
            print('Joint Rehoming Success!')
            # Indicate that Fox encoder has been homed successfully
            self.parent.FLAG_DICT['MAIN_JOINT_ENCODER_HOMED_FLAG'] = True
            self.parent.led_buzzer.play_buzzer(500, 200, 90, "ALL")
            self.parent.led_buzzer.play_buzzer(800, 200, 90, "ALL")
        else:
            print('Joint Rehoming Error!')
            self.parent.led_buzzer.play_buzzer(500, 200, 90, "ALL")
            self.parent.led_buzzer.play_buzzer(200, 200, 90, "ALL")

    def ankle_offset(self, _):
        if self.parent.CAPS13.data[3] == 0x01:
            # subtract offset if 0x01/ plantarflexion
            limit_flex = abs(self.parent.device_parameters.DEG_LIMIT_FLEX)
            if self.parent.CAPS13.data[4] > limit_flex:
                ankle_offset = self.parent.device_parameters.DEG_LIMIT_FLEX
                self.parent.PARAM_DICT['ANKLE_NEUTRAL_OFFSET'][0] = ankle_offset
            else:
                ankle_offset = -self.parent.CAPS13.data[4]
                self.parent.PARAM_DICT['ANKLE_NEUTRAL_OFFSET'][0] = ankle_offset
        elif self.parent.CAPS13.data[3] == 0x02:
            # add offset if 0x02 / dorsiflexion
            limit_ext = self.parent.device_parameters.DEG_LIMIT_EXT
            if self.parent.CAPS13.data[4] > limit_ext:
                ankle_offset = self.parent.device_parameters.DEG_LIMIT_EXT
                self.parent.PARAM_DICT['ANKLE_NEUTRAL_OFFSET'][0] = ankle_offset
            else:
                ankle_offset = self.parent.CAPS13.data[4]
                self.parent.PARAM_DICT['ANKLE_NEUTRAL_OFFSET'][0] = ankle_offset
        self.parent.CAPS13.zero()
        self.parent.report_status('Ankle neutral position changed')

    def reset_controller(self, _):
        self.parent.led_buzzer.play_buzzer(400, 100, 90, "ALL")
        pyb.hard_reset()

    def zero_imu(self, _):
        self.parent.report_status('Zeroing IMU - Leg must be vertical')
        # self.parent.imu.set_orientation()
        self.parent.imu.tare()
        self.parent.CAPS9.zero()
        self.parent.CAPS13.zero()
        self.parent.led_buzzer.play_buzzer(500, 200, 90)
        self.parent.led_buzzer.play_buzzer(800, 200, 90)

    def invert_shank_thigh(self, _):
        self.parent.report_status('Inverting shank/thigh orientation')
        # Invert the shank/thigh orientation
        self.parent.arc_tan2_taylor_coefs[13] = (
            -self.parent.arc_tan2_taylor_coefs[13]
        )
        self.parent.CAPS9.zero()
        self.parent.led_buzzer.play_buzzer(500, 200, 90)
        self.parent.led_buzzer.play_buzzer(800, 200, 90)

    def buzzer_state(self, _):
        # Call to enable buzzer from CAPS10 message
        # Extract frequency, volume, and duration information for beep request.
        # Getting beep frequency stored in CAPS10 data
        frequency = (
            (self.parent.CAPS10.data[1] << 8) | self.parent.CAPS10.data[2]
        )
        # Getting beep volume stored in CAPS10 data
        volume = (
            (self.parent.CAPS10.data[3] << 8) | self.parent.CAPS10.data[4]
        )
        # Getting beep duration stored in CAPS10 data
        duration = (
            (self.parent.CAPS10.data[5] << 8) | self.parent.CAPS10.data[6]
        )
        if frequency > 0:
            # Play beep
            self.parent.led_buzzer.play_buzzer(
                frequency, duration, volume
            )
        else:
            print('Frequency of zero, buzzer cancelled')
        # Clear CAPS10 messages to prevent going back into this a second time
        self.parent.CAPS10.zero()
        
    def zero_fts(self, _):
        self.parent.report_status('Zeroing FTS')
        self.parent.fts_sensor.zero_calibration()
        self.parent.CAPS9.zero()
        self.parent.led_buzzer.play_buzzer(500, 200, 90)
        self.parent.led_buzzer.play_buzzer(800, 200, 90)

