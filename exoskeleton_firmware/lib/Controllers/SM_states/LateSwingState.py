from lib.Controllers.SM_states.StateInterface import StateInterface
from lib.ll_common.error_flag import print_error


class LateSwingState(StateInterface):
    def __init__(self, robot):
        super().__init__(robot)
        self.name = "LateSwing"
        self.dt = 2.0
        self.eps_pos = 10
        self.eps_vel = 10

        self.Fz_tresh=10

    def handle(self):
        if self.first_time:
            self.first_time = False
            print("Late Swing State")
            self.set_KD_parameter(self.K, self.B)
            self.robot.set_transition_point(self.q_des, self.qdot_des, self.dt)
        else: # do nothing (PVT will keep the previous config)
            pass

    def checkConditionForChangingState(self):
        if ((self.robot.get_position() <= self.q_des + self.eps_pos) and
             (self.robot.get_velocity() <= self.eps_vel)): # and (self.robot.load_cell_Fz > self.Fz_tresh)):
            return True
        else:
            return False

    def getNextState(self):
        self.first_time = True # next time it enters it will be again first time
        print("Late swing to stance")
        return "Stance"
