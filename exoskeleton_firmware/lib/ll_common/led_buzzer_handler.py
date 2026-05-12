import time

print('IMPORTING LED BUZZER HANDLER')

class LedBuzzerHandler:
    """
    Manages LED and buzzer hardware control for the HybridLeg system.
    """

    def __init__(self, parent):
        self.parent = parent

        self.led_blink_interval = 700 # msec
        self.led_timer = 0
        self.led_timer_prev_dict = {'EXTERNAL': 0, 'PYBOARD': 0}
        self.led_toggle_dict = {'EXTERNAL': True, 'PYBOARD': True}
        self.led_last_color_dict = {'EXTERNAL': 0, 'PYBOARD': 0}

        self.buzzer_device = None
        self.buzzer_parameters = [0, 0, 0]  # freq, msec, duty_cycle
        self.buzzer_duration = 0 # msec
        self.buzzer_start_time = 0
        self.buzzer_time_checker = 0
        self.error_buzz_timer_previous = 0

    # ========================================================================
    # LED Control Methods
    # ------------------------------------------------------------------------
    def pyboard_LED_rgb(self, r, g, b):
        """
        Sets pyboard RGB LEDs on or off (0=off, 1=on).
        """
        if r:
            self.parent.CONST_DICT['RED_LED'].on()
        else:
            self.parent.CONST_DICT['RED_LED'].off()
        if g:
            self.parent.CONST_DICT['GREEN_LED'].on()
        else:
            self.parent.CONST_DICT['GREEN_LED'].off()
        if b:
            self.parent.CONST_DICT['BLUE_LED'].on()
        else:
            self.parent.CONST_DICT['BLUE_LED'].off()

    def external_LED_rgb(self, r, g, b):
        """
        Sets external RGB LEDs on or off (0=off, 1=on).
        """
        # These external_led variables are defined in hardware_manager.py
        self.parent.external_led_red.value(r)
        self.parent.external_led_green.value(g)
        self.parent.external_led_blue.value(b)

    def update_led(self, led_name, led_color1, led_color2='WHITE',
                   blink=False, blink_interval=None):
        """
        Controls pyboard and external RGB LEDs based on color command.
        If blink is True, alternates between led_color1 and led_color2 (default WHITE).
        """
        color = self.parent.CONST_DICT[led_color1][0]
        r,g,b = self.parent.CONST_DICT[led_color1][1]

        if blink:
            if blink_interval is not None:
                blink_time = blink_interval
            else:
                blink_time = self.led_blink_interval
            self.led_timer = time.ticks_ms()
            if time.ticks_diff(self.led_timer, self.led_timer_prev_dict[led_name]) >= blink_time:
                self.led_timer_prev_dict[led_name] = self.led_timer
                self.led_toggle_dict[led_name] = not self.led_toggle_dict[led_name]
            if self.led_toggle_dict[led_name]:
                # Set r,g,b to led_color2 when time to blink
                color = self.parent.CONST_DICT[led_color2][0]
                r,g,b = self.parent.CONST_DICT[led_color2][1]

        # Only update LED if color command has changed
        if self.led_last_color_dict[led_name] != color:
            self.led_last_color_dict[led_name] = color
            if led_name == 'EXTERNAL':
                self.external_LED_rgb(r, g, b)
                # print(led_name, r, g, b)
            elif led_name == 'PYBOARD':
                self.pyboard_LED_rgb(r, g, b)
                # print(led_name, r, g, b)

    def startup_led_sequence(self):
        """
        Flash PYBOARD LED in red, green, blue sequence to signal program start.
        """
        for _ in range(4):
            self.update_led('PYBOARD', 'RED')
            time.sleep(0.2)
            self.update_led('PYBOARD', 'OFF')
            self.update_led('PYBOARD', 'GREEN')
            time.sleep(0.2)
            self.update_led('PYBOARD', 'OFF')
            self.update_led('PYBOARD', 'BLUE')
            time.sleep(0.2)
            self.update_led('PYBOARD', 'OFF')
        # Set final LED state to red (Default state until boot complete)
        self.update_led('PYBOARD', 'RED')


    # ========================================================================
    # Buzzer Control Methods
    # ------------------------------------------------------------------------
    def play_buzzer(self, freq=0, msec=0, duty_cycle=0, device_to_buzz=None):
        """
        Activate onboard buzzer with frequency (tone), duration,
        and duty cycle (volume).
        """
        if self.parent.components.Buzzer.is_enabled():
            is_primary = self.parent.device_parameters.PRIMARY_CONTROLLER
            is_device = device_to_buzz == self.parent.device_parameters.device
            is_all = device_to_buzz == "ALL"
            if ((device_to_buzz is None and is_primary) or is_device or is_all):
                if self.buzzer_start_time == 0:
                    if duty_cycle > 90:
                        duty_cycle = 90
                    if msec > 2000:
                        msec = 2000
                    self.parent.timer_buzzer_pwm.freq(freq)
                    self.buzzer_duration = msec
                    self.parent.buzzer_ch.pulse_width_percent(duty_cycle)
                    self.buzzer_start_time = time.ticks_ms()
                else:
                    # If the buzzer is already running, store new buzzer values for later
                    params = self.buzzer_parameters
                    params[0] = freq
                    params[1] = msec
                    params[2] = duty_cycle
                    self.buzzer_device = device_to_buzz

    def stop_buzzer(self):
        """
        Deactivate onboard buzzer.
        """
        if self.parent.components.Buzzer.is_enabled():
            self.parent.buzzer_ch.pulse_width_percent(0)
            self.buzzer_start_time = 0
            params = self.buzzer_parameters
            if params[0] > 0:
                self.play_buzzer(
                    params[0],
                    params[1],
                    params[2],
                    self.buzzer_device
                )
            params[0] = 0
            params[1] = 0
            params[2] = 0
            self.buzzer_device = None

    # ========================================================================
    # LED AND BUZZER HIGH LEVEL LOGIC
    # ------------------------------------------------------------------------
    def update_status_indicators(self):
        """
        Logic for controlling the PYBOARD and EXTERNAL LEDs as well as the
        buzzer based on system flags and error conditions.
        This function must be placed inside the main loop and called regularly
        """

        # -------------------------------------------------------------
        #   PYBOARD LED LOGIC
        # -------------------------------------------------------------
        if not self.parent.FLAG_DICT['STREAM_FLAG']:
            self.update_led('PYBOARD', 'BLUE', blink=True)
        elif self.parent.FLAG_DICT['LARGE_MOTOR_STATE']:
            self.update_led('PYBOARD', 'YELLOW')
        elif self.parent.FLAG_DICT['DYNAMIC_BRAKE_STATE']:
            self.update_led('PYBOARD', 'TURQUOISE')
        else:
            self.update_led('PYBOARD', 'GREEN', blink=True)

        # -------------------------------------------------------------
        #   EXTERNAL LED LOGIC
        # -------------------------------------------------------------
        has_error = self.parent.error_flag.contains_true()
        has_low_battery = self.parent.FLAG_DICT['LOW_BATTERY_FLAG']
        if has_error and has_low_battery:
            self.update_led(
                'EXTERNAL', 'MAGENTA', 'RED', blink=True, blink_interval=350
            )
        elif has_error:
            self.update_led(
                'EXTERNAL', 'MAGENTA'
            )
        elif has_low_battery:
            self.update_led(
                'EXTERNAL', 'RED', 'GREEN', blink=True
            )
        else:
            self.update_led('EXTERNAL', 'OFF')

        # -------------------------------------------------------------
        #   BUZZER LOGIC
        # -------------------------------------------------------------
        # Check timer for buzzer handling
        self.buzzer_time_checker = time.ticks_ms()
        # Stop buzzer after duration has elapsed
        if self.buzzer_start_time > 0:
            elapsed_time = time.ticks_diff(
                self.buzzer_time_checker,
                self.buzzer_start_time
            )
            if elapsed_time > self.buzzer_duration:
                self.stop_buzzer()
        # -------------------------------------------------------------
        #   Handle critical battery buzzer
        # -------------------------------------------------------------
        if self.parent.FLAG_DICT['CRITICAL_BATTERY']:
            # Initialize the threshold if it hasn't been correctly set yet
            if self.parent.PARAM_DICT['CRITICAL_BATTERY_BUZZER_THRESHOLD'][0] > 25:
                self.parent.PARAM_DICT['CRITICAL_BATTERY_BUZZER_THRESHOLD'][0] = (
                    int(self.parent.PARAM_DICT['BATTERY_VOLTS'][0] / 0.5) * 0.5
                )

            # Run the critical battery buzzer every 0.5V drop in battery voltage
            if self.parent.PARAM_DICT['CRITICAL_BATTERY_BUZZER_COUNTER'] >= 200:
                buzzer_freq = self.parent.PARAM_DICT['CRITICAL_BATTERY_BUZZER_COUNTER']
                self.play_buzzer(buzzer_freq, 300, 90)
                self.parent.PARAM_DICT['CRITICAL_BATTERY_BUZZER_COUNTER'] -= 1
            else:
                # Threshold gets updated every 0.5V drop
                current_threshold = (
                    int(self.parent.PARAM_DICT['BATTERY_VOLTS'][0] / 0.5) * 0.5
                )
                if (current_threshold <
                    self.parent.PARAM_DICT['CRITICAL_BATTERY_BUZZER_THRESHOLD'][0]
                ):
                    # Battery buzzer is triggered again
                    self.parent.PARAM_DICT['CRITICAL_BATTERY_BUZZER_COUNTER'] = 800
                    self.parent.PARAM_DICT['CRITICAL_BATTERY_BUZZER_THRESHOLD'][0] = (
                        current_threshold
                    )
        # -------------------------------------------------------------
        #   Handle error buzzer
        # -------------------------------------------------------------
        elif self.parent.error_flag.check_if_buzz():
            elapsed_buzz_time = time.ticks_diff(
                self.buzzer_time_checker,
                self.error_buzz_timer_previous
            )
            if elapsed_buzz_time > self.parent.CONST_DICT['ERROR_BUZZ_INTERVAL']:
                self.error_buzz_timer_previous = (
                    self.buzzer_time_checker
                )
                # Beep every 200ms
                self.play_buzzer(
                    800, 100, 90, device_to_buzz="ALL"
                )