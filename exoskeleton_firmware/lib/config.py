####################################################################################################
# @file
#
# @brief            Defines application-specific constants and settings.
####################################################################################################


# ===================================================================================================
# Load in external dependencies
# ---------------------------------------------------------------------------------------------------
# import ll_common as llc                                     # Provides hardware specific layout information.
from ll_common import decorators
from ll_common import routing
import sys

# print('IMPORTING CONFIG - DEVICE SETTINGS')

# =============================================================================
# Identify processor type
# -----------------------------------------------------------------------------
machine = sys.implementation._machine
if machine == 'Arduino Portenta H7 with STM32H747':
    PROCESSOR = 'STM32H747'
elif machine == 'PYBD-SF6W with STM32F767IIK':
    PROCESSOR = 'STM32F767'
else:
    raise ValueError('Unknown processor type: {}'.format(machine))

# ===================================================================================================
# Identify device type
# ---------------------------------------------------------------------------------------------------
# The device ID is an integer read from 'device_id.txt'. This file contains a single integer value and
# nothing else. It is excluded from version control in .gitignore. This allows the same code to control
# all the leg devices without modification.

# In the future, we may wish to explore using the selector switch to set the device ID rather than a
# file.

HYBRID_KNEE                             = 1      # Hybrid Knee device
POLYCENTRIC_ANKLE                       = 2      # Polycentric device
POLYCENTRIC_ANKLE_ONLY                  = 3      # Polycentric device only, used as primary, no knee attached
FOX_KNEE                                = 8      # Fox Knee device

config_device = 0

####################################################################################################
# @class            DeviceParameters
#
# @brief            Defines which configurations are enabled depending on the device
####################################################################################################


class DeviceParameters():
    def __init__(self, device):
        global config_device
        config_device = device
        self.device = device

        self.PROCESSOR = PROCESSOR

        # ---------------------------------------------------------------------
        #
        #   HYBRID KNEE
        #
        # ---------------------------------------------------------------------
        if self.device == HYBRID_KNEE:                                  # variables related to hybrid knee
            self.DEG_LIMIT_FLEX                      = 108              # measured flexion limit in deg
            self.DEG_LIMIT_EXT                       = 0                # measured extension limit in deg
            self.ROTATION_DIRECTION                  = 1               # Indicates the direction of rotation: -1 for hybrid knee and 1 for p2 ankle
            self.PRIMARY_CONTROLLER                  = True             # This declares the hybrid knee as the primary controller for the pyboard statemachine

            if self.PROCESSOR == 'STM32H747':
                self.TIMER_ROBOT = {
                    'ID': 1,
                    'FREQ': 200
                }
                self.TIMER_BRAKING_PWM = {
                    'ID': 8,
                    'FREQ': 78000,
                    'CHANNEL': 3
                }
                self.TIMER_BUZZER_PWM = {
                    'ID': 3,
                    'FREQ': 4000,
                    'CHANNEL': 1
                }

                self.TIMER_SERVO = {
                    'ID': 12,
                    'FREQ': 300
                }

                self.TIMER_CONTROLLER = {
                    'ID': 2,
                    'FREQ': 50
                }
                self.TIMER_REPORTER = {  # output manager
                    'ID': 7,
                    'FREQ': 100
                }
                self.TIMER_INPUT_MANAGER = {
                    'ID': 13,
                    'FREQ': 50
                }


                # From stm32h747xx.h:
                self.NVIC_REGISTERS = {
                    'FDCAN1_IT0_IRQn': 19,          # CAN 1 Line 0
                    'FDCAN1_IT1_IRQn': 21,          # CAN 1 Line 1
                    'TIM8_CC_IRQn': 46,             # Braking PWM Timer
                    'LPTIM3_IRQn': 139,             # Buzzer PWM Timer
                    'TIM6_DAC_IRQn': 54,            # CAN Bus External Timer
                    'LPTIM2_IRQn': 138,             # CAPS Send Timer
                    'TIM8_UP_TIM13_IRQn': 44,       # State Machine Timer
                    'TIM4_IRQn': 30,                # IMU Timer
                    'TIM8_BRK_TIM12_IRQn': 43,      # Main Loop Timer
                    'TIM7_IRQn': 55                 # Motor Control Timer
                }

            elif self.PROCESSOR == 'STM32F767':
                self.TIMER_BRAKING_PWM = {
                    'ID': 1,
                    'FREQ': 78000,
                    'CHANNEL': 3
                }
                self.TIMER_BUZZER_PWM = {
                    'ID': 5,
                    'FREQ': 4000,
                    'CHANNEL': 1
                }
                self.TIMER_CAN_EXTERNAL = {
                    'ID': 6,
                    'FREQ': 1000
                }
                self.TIMER_CAPS_SEND = {
                    'ID': 2,
                    'FREQ': 250
                }
                self.TIMER_STATE_MACHINE = {
                    'ID': 13,
                    'FREQ': 100
                }
                self.TIMER_IMU = {
                    'ID': 4,
                    'FREQ': 50
                }
                self.TIMER_MAIN_LOOP = {
                    'ID': 7,
                    'FREQ': 250
                }
                self.TIMER_MOTOR_CONTROL = {
                    'ID': 12,
                    'FREQ': 500
                }

                self.NVIC_REGISTERS = {
                    'CAN1_RX0_IRQn': 20,            # CAN 1 RX0
                    'CAN1_RX1_IRQn': 21,            # CAN 1 RX1
                    'CAN1_SCE_IRQn': 22,            # CAN 1 SCE
                    'TIM5_IRQn': 50,                # Buzzer PWM Timer
                    'TIM2_IRQn': 28,                # CAPS Send Timer
                    'TIM7_IRQn': 55,                # Main Loop Timer
                    'TIM12_IRQn': 43                # Motor Control Timer
                }

            print('INITIALIZING HYBRID KNEE PARAMETERS')

        # ---------------------------------------------------------------------
        #
        #   FOX KNEE
        #
        # ---------------------------------------------------------------------
        elif self.device == FOX_KNEE:

            # flexion limit in degrees
            self.DEG_LIMIT_FLEX = 108

            # extension limit in deg
            self.DEG_LIMIT_EXT = 0

            # Indicates the direction of motor rotation
            self.ROTATION_DIRECTION = 1               

            # whether device is the primary controller for the pyboard
            # statemachine
            self.PRIMARY_CONTROLLER = True

            #
            #   TIMER SETTINGS
            #

            # Assume processor is H747 (Portenta H7 boards)
            self.TIMER_BRAKING_PWM = {
                'ID': 8,
                'FREQ': 78000,
                'CHANNEL': 3
            }
            self.TIMER_BUZZER_PWM = {
                'ID': 3,
                'FREQ': 4000,
                'CHANNEL': 1
            }
            self.TIMER_ROBOT = {
                'ID': 1,
                'FREQ': 200
            }
            self.TIMER_LOADCELL = {
                'ID': 6,
                'FREQ': 200
            }
            ## TINMER 5 crash the system
            # TIMER 9, 10, 11 doesn't exist
            self.TIMER_FSR = {
                'ID': 14,
                'FREQ': 100
            }
            self.TIMER_IMU = {
                'ID': 4,
                'FREQ': 200
            }
            self.TIMER_SERVO = {
                'ID': 12,
                'FREQ': 300
            }
            self.TIMER_CONTROLLER = {
                'ID': 2,
                'FREQ': 50
            }
            self.TIMER_REPORTER = { # output manager
                'ID': 7,
                'FREQ':100
            }
            self.TIMER_INPUT_MANAGER = {
                'ID': 13,
                'FREQ': 50
            }

            # From stm32h747xx.h:
            self.NVIC_REGISTERS = {
                'FDCAN1_IT0_IRQn': 19,          # CAN 1 Line 0
                'FDCAN1_IT1_IRQn': 21,          # CAN 1 Line 1
                'TIM8_CC_IRQn': 46,             # Braking PWM Timer
                'LPTIM3_IRQn': 139,             # Buzzer PWM Timer
                'TIM6_DAC_IRQn': 54,            # CAN Bus External Timer
                'LPTIM2_IRQn': 138,             # CAPS Send Timer
                'TIM8_UP_TIM13_IRQn': 44,       # State Machine Timer
                'TIM4_IRQn': 30,                # IMU Timer
                'TIM8_BRK_TIM12_IRQn': 43,      # Main Loop Timer
                'TIM7_IRQn': 55                 # Motor Control Timer
            }

            print('INITIALIZING FOX KNEE PARAMETERS')

        # ---------------------------------------------------------------------
        #
        #   POLYCENTRIC ANKLE
        #
        # ---------------------------------------------------------------------
        elif self.device == POLYCENTRIC_ANKLE:                          # Variables related to polycentric ankle
            self.DEG_LIMIT_FLEX                      = -20              # measured plantarflexion limit in deg - without added end stop, -27; with added end stop, -15
            self.DEG_LIMIT_EXT                       = 20               # measured dorsiflexion limit in deg
            self.ROTATION_DIRECTION                  = 1                # Indicates the direction of rotation: 1 for hybrid knee and -1 for p2 ankle
            self.PRIMARY_CONTROLLER                  = False            # This declares the hybrid knee as the primary controller for the pyboard statemachine
            self.PROCESSOR = PROCESSOR

            print('INITIALIZING POLYCENTRIC ANKLE PARAMETERS')

            if self.PROCESSOR == 'STM32H747':
                self.TIMER_BUZZER_PWM = {
                    'ID': 3,
                    'FREQ': 4000,
                    'CHANNEL': 1
                }
                self.TIMER_CAN_EXTERNAL = {
                    'ID': 6,
                    'FREQ': 1000
                }
                self.TIMER_CAPS_SEND = {
                    'ID': 2,
                    'FREQ': 250
                }
                self.TIMER_STATE_MACHINE = {
                    'ID': 13,
                    'FREQ': 100
                }
                self.TIMER_IMU = {
                    'ID': 4,
                    'FREQ': 50
                }
                self.TIMER_MAIN_LOOP = {
                    'ID': 12,
                    'FREQ': 500
                }
                self.TIMER_MOTOR_CONTROL = {
                    'ID': 7,
                    'FREQ': 250
                }

                # From stm32h747xx.h:
                self.NVIC_REGISTERS = {
                    'FDCAN1_IT0_IRQn': 19,          # CAN 1 Line 0
                    'FDCAN1_IT1_IRQn': 21,          # CAN 1 Line 1
                    'LPTIM3_IRQn': 139,             # Buzzer PWM Timer
                    'TIM6_DAC_IRQn': 54,            # CAN Bus External Timer
                    'LPTIM2_IRQn': 138,             # CAPS Send Timer
                    'TIM8_UP_TIM13_IRQn': 44,       # State Machine Timer
                    'TIM4_IRQn': 30,                # IMU Timer
                    'TIM8_BRK_TIM12_IRQn': 43,      # Main Loop Timer
                    'TIM7_IRQn': 55                 # Motor Control Timer
                }

            elif self.PROCESSOR == 'STM32F767':
                self.TIMER_BUZZER_PWM = {
                    'ID': 5,
                    'FREQ': 4000,
                    'CHANNEL': 1
                }
                self.TIMER_CAN_EXTERNAL = {
                    'ID': 6,
                    'FREQ': 1000
                }
                self.TIMER_CAPS_SEND = {
                    'ID': 2,
                    'FREQ': 250
                }
                self.TIMER_STATE_MACHINE = {
                    'ID': 13,
                    'FREQ': 100
                }
                self.TIMER_IMU = {
                    'ID': 4,
                    'FREQ': 50
                }
                self.TIMER_MAIN_LOOP = {
                    'ID': 7,
                    'FREQ': 250
                }
                self.TIMER_MOTOR_CONTROL = {
                    'ID': 12,
                    'FREQ': 500
                }

                self.NVIC_REGISTERS = {
                    'CAN1_RX0_IRQn': 20,            # CAN 1 RX0
                    'CAN1_RX1_IRQn': 21,            # CAN 1 RX1
                    'CAN1_SCE_IRQn': 22,            # CAN 1 SCE
                    'TIM5_IRQn': 50,                # Buzzer PWM Timer
                    'TIM2_IRQn': 28,                # CAPS Send Timer
                    'TIM7_IRQn': 55,                # Main Loop Timer
                    'TIM12_IRQn': 43                # Motor Control Timer
                }
                
        # ---------------------------------------------------------------------
        #
        #   POLYCENTRIC ANKLE ONLY
        #
        # ---------------------------------------------------------------------
        elif self.device == POLYCENTRIC_ANKLE_ONLY:                     # Variables related to polycentric ankle
            self.DEG_LIMIT_FLEX                      = -20              # measured plantarflexion limit in deg - without added end stop, -27; with added end stop, -15
            self.DEG_LIMIT_EXT                       = 20               # measured dorsiflexion limit in deg
            self.ROTATION_DIRECTION                  = 1                # Indicates the direction of rotation: 1 for hybrid knee and -1 for p2 ankle
            self.PRIMARY_CONTROLLER                  = True             # This declares the Polycentric Ankle as the primary controller for the pyboard statemachine
            self.PROCESSOR = PROCESSOR

            print('INITIALIZING POLYCENTRIC ANKLE PARAMETERS')

            if self.PROCESSOR == 'STM32H747':
                self.TIMER_BUZZER_PWM = {
                    'ID': 3,
                    'FREQ': 4000,
                    'CHANNEL': 1
                }
                self.TIMER_CAN_EXTERNAL = {
                    'ID': 6,
                    'FREQ': 1000
                }
                self.TIMER_CAPS_SEND = {
                    'ID': 2,
                    'FREQ': 250
                }
                self.TIMER_STATE_MACHINE = {
                    'ID': 13,
                    'FREQ': 100
                }
                self.TIMER_IMU = {
                    'ID': 4,
                    'FREQ': 50
                }
                self.TIMER_MAIN_LOOP = {
                    'ID': 12,
                    'FREQ': 500
                }
                self.TIMER_MOTOR_CONTROL = {
                    'ID': 7,
                    'FREQ': 250
                }

                # From stm32h747xx.h:
                self.NVIC_REGISTERS = {
                    'FDCAN1_IT0_IRQn': 19,          # CAN 1 Line 0
                    'FDCAN1_IT1_IRQn': 21,          # CAN 1 Line 1
                    'LPTIM3_IRQn': 139,             # Buzzer PWM Timer
                    'TIM6_DAC_IRQn': 54,            # CAN Bus External Timer
                    'LPTIM2_IRQn': 138,             # CAPS Send Timer
                    'TIM8_UP_TIM13_IRQn': 44,       # State Machine Timer
                    'TIM4_IRQn': 30,                # IMU Timer
                    'TIM8_BRK_TIM12_IRQn': 43,      # Main Loop Timer
                    'TIM7_IRQn': 55                 # Motor Control Timer
                }

            elif self.PROCESSOR == 'STM32F767':
                self.TIMER_BUZZER_PWM = {
                    'ID': 5,
                    'FREQ': 4000,
                    'CHANNEL': 1
                }
                self.TIMER_CAN_EXTERNAL = {
                    'ID': 6,
                    'FREQ': 1000
                }
                self.TIMER_CAPS_SEND = {
                    'ID': 2,
                    'FREQ': 250
                }
                self.TIMER_STATE_MACHINE = {
                    'ID': 13,
                    'FREQ': 100
                }
                self.TIMER_IMU = {
                    'ID': 4,
                    'FREQ': 50
                }
                self.TIMER_MAIN_LOOP = {
                    'ID': 7,
                    'FREQ': 250
                }
                self.TIMER_MOTOR_CONTROL = {
                    'ID': 12,
                    'FREQ': 500
                }

                self.NVIC_REGISTERS = {
                    'CAN1_RX0_IRQn': 20,            # CAN 1 RX0
                    'CAN1_RX1_IRQn': 21,            # CAN 1 RX1
                    'CAN1_SCE_IRQn': 22,            # CAN 1 SCE
                    'TIM5_IRQn': 50,                # Buzzer PWM Timer
                    'TIM2_IRQn': 28,                # CAPS Send Timer
                    'TIM7_IRQn': 55,                # Main Loop Timer
                    'TIM12_IRQn': 43                # Motor Control Timer
                }

####################################################################################################
# @class            Board
#
# @brief            Defines which hardware components are enabled.
#
# @details          Instantiates the desired routing class (e.g. BoardName_Version_Info) from
#                   the routing.py file in ll_common.
#
#                   @par Once instantiated, enables the desired hardware components (all disabled
#                   by default).
#
# @warning          The decorator used ensures only a single instance of this class is used across
#                   all Python code (singleton implementation).
#
# @note             This is not an initialization class, rather, it defines the set of componenents
#                   on the hardware layer which are considered available to the appliation layer.
#
# @note             You can change the default protocol for any component by adding a line using
#                   the following syntax to the constructor:
#                       * self._components.[COMPONENT].protocol = llc.routing.Protocol.[PROTOCOL]
#
#                   This is relevant if you are experimenting with say I2C vs SPI for communicating
#                   with an IC that has both options routed.
#
#                   @par You will likely need to also modify the hardware as well as add the line
#                   here. Ideally, if this is to be permanent, we should modify the constructor for
#                   your routing class in the ll_common library.
####################################################################################################


@decorators.singleton
class Board(object):

    ################################################################################################
    # @brief        Initialization function for class.
    #
    # @param[in]    self                Object pointer to class instance.
    ################################################################################################
    def __init__(self):

        # Create instance of desired routing class.Change the following lines to switch between
        # hardware platforms and revisions:
        if PROCESSOR == 'STM32H747':
            if config_device == HYBRID_KNEE:
                self._components = routing.UPythonLLCtrl_vB_2_0()
            elif config_device == FOX_KNEE:
                self._components = routing.UPythonLLCtrl_vB_3_0()
        elif PROCESSOR == 'STM32F767':                                               # pyboard
            self._components = routing.UPythonLLCtrl_vB_1_1()
        print('config device', config_device)

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # The following lists contains any and all components that can be enabled. By default,
        # all entries are disabled. To enable any component, call its related enable() method.
        #
        # self._components.AuxiliaryAnalogIn.enable()       # Enable this to enable an additional analog in
        # self._components.AuxiliaryComm.enable()           # Enable this to allow aux communications
        # self._components.Barometer.enable()               # Enable this to access baromter readings
        # self._components.BrakingCircuit.enable()          # Enable this to enable the braking circuit
        # self._components.Buzzer.enable()                  # Enable this to control the buzzer on the controller board
        # self._components.Elmo.enable()                    # Enable this to control large motors (usually the main joint)
        # self._components.Everest.enable()                 # servo made by Novanta/Ingenia
        # self._components.ExternalComm.enable()            # Enable this to allow for CAN communication
        # self._components.ExternalLEDs.enable()            # Enable this to control the LED on the controller board (not pyboard)
        # self._components.IMU.enable()                     # Enable this to get IMU values
        # self._components.InternalLEDs.enable()            # Enable this to enable pyboard LEDs (not really necessary)
        # self._components.JointEncoder.enable()            # Enable this to enable the absoulute encoder usually associated withe main joint
        # self._components.ModeSelectionSwitch.enable()     # Enable this to enable the switch that controls the mode of the code for each tunable_settings.device - not active
        # self._components.QwiicConnector.enable()          # Enable this to communicate with peripheral devices attached vai the Qwiic connector.
        # self._components.SafetySwitch.enable()            # Enable this to enable the safety switch - not active
        # self._components.TimingOutput.enable()            # Output pin on debug header for timing code sections
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        if config_device == HYBRID_KNEE:
            self._components.BrakingCircuit.enable()
            self._components.Buzzer.enable()
            self._components.Everest.enable()
            self._components.ExternalComm.enable()
            self._components.ExternalLEDs.enable()
            self._components.IMU.enable()
            self._components.InternalLEDs.enable()
            self._components.ModeSelectionSwitch.enable()
            # self._components.QwiicConnector.enable()
            self._components.TimingOutput.enable()

        elif config_device == FOX_KNEE:
            self._components.BrakingCircuit.enable()
            self._components.Buzzer.enable()
            self._components.Everest.enable()
            self._components.ExternalComm.enable()
            self._components.ExternalLEDs.enable()
            self._components.IMU.enable()
            self._components.InternalLEDs.enable()
            self._components.ModeSelectionSwitch.enable()
            # self._components.QwiicConnector.enable()
            self._components.TimingOutput.enable()

        elif config_device == POLYCENTRIC_ANKLE:
            self._components.Buzzer.enable()
            self._components.Everest.enable()
            self._components.ExternalComm.enable()
            self._components.ExternalLEDs.enable()
            self._components.InternalLEDs.enable()
            self._components.ModeSelectionSwitch.enable()
            self._components.TimingOutput.enable()

        elif config_device == POLYCENTRIC_ANKLE_ONLY:
            self._components.Buzzer.enable()
            self._components.Everest.enable()
            self._components.ExternalComm.enable()
            self._components.ExternalLEDs.enable()
            self._components.IMU.enable()
            self._components.InternalLEDs.enable()
            self._components.ModeSelectionSwitch.enable()
            self._components.TimingOutput.enable()

        else:
            print('NO VALID DEVICE TYPE DECLARED FOR CONFIG.PY')

    # ###############################################################################################
    # @brief        Initialization function for class.
    #
    # @retval       components          Copy of the internal instance of the desired routing class
    #                                   for the target PCB fully instantiated.
    ################################################################################################

    def get_components(self):
        return self._components
