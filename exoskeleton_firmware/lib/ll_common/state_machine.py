# This script handles the state-machine
#
#
# Author: Levi Hargrove
# Sept 29, 2021
# Revised Nov 7, 2021
#
# This file takes loads in a .psm file that was created using a Slave script in CAPS (create_psm.py).
# Put the create_psm.py is an addafter slave in after the .daq input process. Drag and drop the created file
# into the pyboard main folder. The __init__ file will load the file, passed in as f_name, and create all the
# variables that is required for use by the state machine.

import array
import struct
import time

from uctypes import addressof                       # type: ignore

from . import sm_assembler_functions

print('IMPORTING STATE MACHINE')


class StateMachine:
    # Initialization function. Set up variables, register callbacks, etc.

    def __init__(self, sm_channels, sm_max, sm_min, slope_chans, f_name):

        # Load in all of the information required to generate the state machine
        self.f_name = f_name
        self.f1 = open(f_name, 'rb')
        states = struct.unpack('h', self.f1.read(2))                       # Read the number of states in the file
        self.max_states = states[0]
        num_DOFs = struct.unpack('h', self.f1.read(2))                     # The number of DOFs in the file
        self.DOFs = num_DOFs[0]
        starting_state = struct.unpack('h', self.f1.read(2))               # Read the starting state
        self.current_state = starting_state[0]
        print(self.current_state)
        dum = struct.unpack('h', self.f1.read(2))                          # Read the number of events in the file
        self.max_events = dum[0]
        dum = struct.unpack('h', self.f1.read(2))                          # Read the number of events in the file
        self.active_events = dum[0]

        self.event_id = array.array('i', [0] * self.max_events)             # Each event is given an id numbers
        self.state_from = array.array('i', [0] * self.max_events)
        self.state_to = array.array('i', [0] * self.max_events)
        self.event_priority = array.array('i', [0] * self.max_events)
        self.event_enabled = array.array('i', [0] * self.max_events)
        self.event_output = array.array('i', [0] * 650)

        for i in range(self.max_events):						# This loads through all the events and gets
            dum = struct.unpack('h', self.f1.read(2))			# and maps where the come from, and where they go to
            self.state_from[i] = dum[0]							# determines if they are enabled, and what their priority
            dum = struct.unpack('h', self.f1.read(2))			# is. This should all come from CAPS.
            self.state_to[i] = dum[0]
            dum = struct.unpack('h', self.f1.read(2))
            self.event_priority[i] = dum[0]
            dum = struct.unpack('h', self.f1.read(2))
            self.event_enabled[i] = dum[0]

        self.state_info = []
        self.event_list_arrays = []
        self.state_list_arrays = []
        for i in range(self.max_states):
            dum = struct.unpack('h', self.f1.read(2))
            active_state = dum[0]
            self.state_info.append(active_state)
            dum = struct.unpack('h', self.f1.read(2))
            num_list_events = dum[0]
            self.event_list_arrays.append(array.array('f', [0] * num_list_events))
            for j in range(num_list_events):
                dum = struct.unpack('f', self.f1.read(4))
                self.event_list_arrays[i][j] = dum[0]
            dum = struct.unpack('h', self.f1.read(2))
            num_state_events = dum[0]
            self.state_list_arrays.append(array.array('f', [0] * num_state_events))
            for j in range(num_state_events):
                dum = struct.unpack('f', self.f1.read(4))
                self.state_list_arrays[i][j] = dum[0]

        self.dof_id_map = array.array('i', [0] * self.DOFs)
        for j in range(self.DOFs):
            dum = struct.unpack('f', self.f1.read(4))
            self.dof_id_map[j] = int(dum[0])

        self.f1.close()

        self.state_dictionary = {}							# This maps the output of the variable to the
        for i in range(len(self.state_info)):				# correct state
            self.state_dictionary[self.state_info[i]] = i

        self.dof_outputs = array.array('f', [0] * self.DOFs)
        self.num_DOFs_array = array.array('i', [self.DOFs])
        self.sm_state_dofs = array.array('i', [addressof(self.num_DOFs_array), addressof(self.dof_outputs)])

        # The timer callback
        self.timer_val = None
        self.started = False

        # These are variables that continuously change from the sensors on the leg, or modules attached to the leg.
        self.current_chans = sm_channels
        self.max_chans = sm_max
        self.min_chans  = sm_min
        self.slope_chans = slope_chans
        self.state_entry_chans = array.array('f', [0] * len(sm_channels))
        self.prev_chans = array.array('f', [0] * len(sm_channels))
        self.timing_vars = array.array('f', [0, 0, 0, 0])                           # Timing since entering state, frame counter, manual state override, and state_change_var
        self.sensor_addresses = array.array('i', [addressof(self.current_chans), addressof(self.prev_chans), addressof(self.slope_chans), addressof(self.max_chans), addressof(self.min_chans), addressof(self.state_entry_chans), addressof(self.timing_vars)])
        self.state_start_time = time.ticks_ms()
        self.state_frame_counter = 0
        self.active_state = array.array('i', [self.current_state])
        self.subevent_inputs = array.array('i', [addressof(self.event_list_arrays[self.state_dictionary[self.current_state]]), addressof(self.event_enabled), addressof(self.state_from), addressof(self.state_to), addressof(self.event_priority), addressof(self.active_state)])
        self.sm_time_val = 0
        self.sm_dictionary_index = self.state_dictionary[self.current_state]
        self.sm_variable_indexes = array.array('i', [addressof(self.timing_vars),		# 0
                                                     addressof(self.current_chans),		# 4
                                                     addressof(self.max_chans),			# 8
                                                     addressof(self.min_chans),			# 12
                                                     addressof(self.prev_chans),			# 16
                                                     addressof(self.slope_chans)]) 		# 20

    def start_machine(self):
        self.started = True

    def stop_machine(self):
        self.started = False

    def attach_timer(self, timer_val):
        self.timer_val = timer_val
        self.timer_val.callback(self.execute_state_machine)
        self.started = False

    # Runs all the steps required for the statemachine and updates the impedance parameters.
    def execute_state_machine(self, timer):
        if self.started:
            t1 = time.ticks_us()
            time_in_state = time.ticks_diff(time.ticks_ms(), self.state_start_time)
            self.state_frame_counter = self.state_frame_counter + 1
            old_state = self.active_state[0]
            sm_assembler_functions.update_sm_input_vars(self.sm_variable_indexes,
                                                        len(self.current_chans),
                                                        time_in_state,
                                                        self.state_frame_counter)
            sm_assembler_functions.check_sub_events(self.subevent_inputs,
                                                    self.subevent_inputs[0] + 4 * len(self.event_list_arrays[self.sm_dictionary_index]),
                                                    self.event_output,
                                                    self.sensor_addresses)
            sm_assembler_functions.check_states(self.state_list_arrays[self.sm_dictionary_index],
                                                self.sensor_addresses,
                                                0,
                                                self.sm_state_dofs)
            self.current_state = self.active_state[0]
            if old_state != self.current_state:
                self.state_start_time = time.ticks_ms()
                self.state_frame_counter = 0
                print("DETECTED STATE CHANGE!", old_state, self.current_state)
                sm_assembler_functions.state_change_cleanup(self.sensor_addresses, len(self.prev_chans))
                self.sm_dictionary_index = self.state_dictionary[self.current_state]
                self.subevent_inputs[0] = addressof(self.event_list_arrays[self.sm_dictionary_index])

            t2 = time.ticks_us()
            self.sm_time_val = time.ticks_diff(t2, t1)
