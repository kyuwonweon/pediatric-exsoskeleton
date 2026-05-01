import micropython                                  # type: ignore
import pyb, time                                          # type: ignore

from lib.Controllers.ControllerInterface import Controller
from lib.Controllers.ctrlFFTransp import ctrl_FF_Transp
from lib.Controllers.ctrlFBTransp_Swing import ctrl_FB_Transp
from lib.Controllers.ctrlOscillatorPos import ctrlOscillatorPos
from lib.Controllers.ctrlOscillatorForces import ctrl_Oscillator_Forces
from lib.Controllers.ctrlSMWalking import CtrlSMWalking
from lib.Controllers.ctrlTelop import ctrl_Teleop
from lib.Robots.RobotInterface import Robot
# from lib.IO.InputManagerInterface import InputManagerInterface

import json

from lib.config import DeviceParameters

"""
Ctrl factory that manage the multiple controllers depending by the input messages
"""

class CtrlFactory:
    def __init__(self, robot:Robot,
                 file_name:str):
        self.robot = robot
        self.file_name = file_name

        self.device_parameters = robot.device_parameters

        self._registryNameToConstructor={"ctrlInterface": Controller,
                    # "ctrlFFTransp": ctrl_FF_Transp,
                    # "ctrlFBTranspSwing": ctrl_FB_Transp,
                    # "ctrlTelop": ctrl_Teleop,
                    "ctrlSMWalking": CtrlSMWalking,
                    # "ctrlOscillatorForce": ctrl_Oscillator_Forces,
                    # "ctrlOscillatorPos": ctrlOscillatorPos,
                                         }

        with open(self.file_name, "r") as f:
            self.params = json.load(f)


        self.available_ctrl = self.params["ctrlFactory"]["available_ctrl"]
        self.default_ctrl = self.params["ctrlFactory"]["default_ctrl"]
        # default controller is the controller interface that it is idle
        print(f"default ctrl: {self.default_ctrl}")
        self.ctrl = self._registryNameToConstructor[self.default_ctrl](self.robot,
                                                                       self.file_name,
                                                                       self.default_ctrl,
                                                                       device_params= self.device_parameters)
        # second registry from headers to name
        self._registryHeaderToName ={}
        for ctrl_name in self.available_ctrl:
            h = self.params[ctrl_name]["header"]
            self._registryHeaderToName[h] = ctrl_name

        # reporter dimensions
        self.rep_ctrl_fact_msg_dim = self.params["reporter"]['dim_ctrl_fact_msg']
        self.rep_ctrl_fact_msg = [self.ctrl.ctrl_header]* self.rep_ctrl_fact_msg_dim

        # input manager msgs
        self.dim_robot_cal_mgs = self.params["input_manager"]['dim_robot_cmd']
        self.dim_ctrl_input_msg = self.params["input_manager"]['dim_ctrl_cmd']
        self.dim_ctrl_fact_input_msg = self.params["input_manager"]['dim_ctrl_fact_cmd']
        self.dim_msg = (self.dim_robot_cal_mgs + self.dim_ctrl_input_msg + self.dim_ctrl_fact_input_msg)

    def updateCtrl(self, ctrl_index) -> bool:
        print(f"update ctrl, current: {self._registryHeaderToName[self.ctrl.ctrl_header]},",
              f" new: {self._registryHeaderToName[ctrl_index]}" )
        ## disable ctrl first
        self.ctrl.disable()
        # change controller
        # TODO:the internal check of ctrl to name doesn't work
        self.ctrl = self._registryNameToConstructor[self._registryHeaderToName[ctrl_index]](self.robot,
                                                                       self.file_name,
                                                                       self._registryHeaderToName[ctrl_index],
                                                                       device_params= self.device_parameters)
        ## verify ctrl has been changed
        if ctrl_index != self.ctrl.ctrl_header:
            print("Ctrl update failed")
            return False
        return True

    def processInputs(self, params:list[float] ,index =0) -> bool:
        if len(params) != self.dim_msg:
            raise Exception('Invalid number of parameters')

        # first part of the input is used to check that I'm using the right controller
        if params[index + 0] == self.ctrl.ctrl_header:
            index += self.dim_ctrl_fact_input_msg
            # pass the rest to the controller for cascade processing
            if not self.ctrl.set_params_ctrl(params, index): return False
        else:
            # change controller
            if not self.updateCtrl(params[0]):  return False
            print("Ctrl update succeded: ", self.ctrl.ctrl_header)
            index += self.dim_ctrl_fact_input_msg
            if not self.ctrl.set_params_ctrl(params, index): return False
        return True

    def create_report(self, out, index) -> int:
        ## factory report has dimension 1
        out[index]= self.ctrl.ctrl_header
        index += self.rep_ctrl_fact_msg_dim
        return self.ctrl.create_report(out, index)





