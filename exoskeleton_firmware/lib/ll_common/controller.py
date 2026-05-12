##############################################################################################
# @file       controller.py
#
# @details    Motor control code for hybrid leg
#
# @author     Jim Lipsey
##############################################################################################

import array
import time

from micropython import const                       # type: ignore

#
#   Constants for referencing array positions in
#   Controller.controller_parameters
#
ERROR                       = const(0)
B                           = const(1)
DELTA_ANGLE                 = const(2)
D                           = const(3)
DELTA_ERROR                 = const(4)
DELTA_TIME                  = const(5)
KT                          = const(6)
OLD_ERROR                   = const(7)
OLD_ANGLE                   = const(8)
FEEDBACK_VALUE              = const(9)
SET_POINT                   = const(10)
ONE_MILLION                 = const(11)
K                           = const(12)

# Most of these seem to be unused, however may still be in CAN mapping, along
# with T_BIAS

# POSITION_ESTIMATE           = const(13)
# PREDICTED_POSITION_ERROR    = const(14)
VELOCITY_ESTIMATE           = const(15)
# VELOCITY_INTEGRATOR         = const(16)
# KP_TRACKING_LOOP            = const(17)
# KI_TRACKING_LOOP            = const(18)

T_BIAS                      = const(19)     # position hard-coded in CAN setup


class Controller():                                       # PD Controller
    def __init__(self, K_p=0.0, K_b=0.0, K_d=0.0, K_t=0.0, T_bias=0.0):

        self.last_time = time.ticks_us()
        self.current_time = time.ticks_us()

        self.output_array = array.array('f', [0.0, 0.0, 0, 0])

        self.controller_parameters = array.array('f', 20 * [0.0])
        self.controller_parameters[ERROR] = 0.0
        self.controller_parameters[B] = K_b
        self.controller_parameters[DELTA_ANGLE] = 0.0
        self.controller_parameters[D] = K_d
        self.controller_parameters[DELTA_ERROR] = 0.0
        self.controller_parameters[DELTA_TIME] = 0.0
        self.controller_parameters[KT] = K_t
        self.controller_parameters[OLD_ERROR] = 0.0
        self.controller_parameters[OLD_ANGLE] = 0.0
        self.controller_parameters[FEEDBACK_VALUE] = 0.0
        self.controller_parameters[SET_POINT] = 0.0
        self.controller_parameters[ONE_MILLION] = 1000000.0
        self.controller_parameters[K] = K_p

        self.controller_parameters[T_BIAS] = T_bias

    def clear_all(self):
        self.controller_parameters[SET_POINT] = 0
        self.controller_parameters[K] = 0
        self.controller_parameters[B] = 0
        self.controller_parameters[D] = 0
        self.controller_parameters[OLD_ERROR] = 0
        self.controller_parameters[OLD_ANGLE] = 0
        self.controller_parameters[T_BIAS] = 0

    def clear_parameters_with_damping_minimum(self):
        self.controller_parameters[SET_POINT] = 0
        self.controller_parameters[K] = 0
        self.controller_parameters[B] = 0.1
        self.controller_parameters[D] = 0
        self.controller_parameters[OLD_ANGLE] = 0
        self.controller_parameters[OLD_ERROR] = 0
        self.controller_parameters[T_BIAS] = 0

    def controller_call(self, feedback_array, velocity_array):
        # Getting the data from the feedback_array and inserting it into the
        # parameter list
        controller_params = self.controller_parameters
        output_array = self.output_array
        controller_params[FEEDBACK_VALUE] = feedback_array[0]
        
        self.current_time = time.ticks_us()
        ticks_diff = time.ticks_diff(self.current_time, self.last_time)

        if (
            ticks_diff > 5000
            or ticks_diff < 0
        ):
            controller_params[DELTA_TIME] = 5000/ 1E6
            # print('Long Controller Tick Time', 'ticks diff = ', ticks_diff)
        else:
            controller_params[DELTA_TIME] = ticks_diff / 1E6
        self.last_time = self.current_time

        # Calculate error. Store error for next calculation
        error_val = (controller_params[SET_POINT]
                     - controller_params[FEEDBACK_VALUE])
        
        delta_error = error_val - controller_params[OLD_ERROR]
        controller_params[OLD_ERROR] = error_val

        controller_params[OLD_ANGLE] = controller_params[FEEDBACK_VALUE]
        
        output_array[0] = (
            controller_params[K] * error_val
            - controller_params[B] * velocity_array[0]
            - controller_params[D] * delta_error / controller_params[DELTA_TIME]
            + controller_params[T_BIAS]
        ) / controller_params[KT]
