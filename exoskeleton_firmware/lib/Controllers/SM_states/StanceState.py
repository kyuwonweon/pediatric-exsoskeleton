from lib.Controllers.SM_states.StateInterface import StateInterface
import time

class StanceState(StateInterface):
    def __init__(self, robot):
        super().__init__(robot)
        self.name = "Stance"
        self.t0 = time.ticks_us()
        self.eps_pos = 10
        self.eps_vel = 5

        self.theta_thigh_thresh =0
        self.Fz_tresh =10

    def handle(self):
        if self.first_time:
            self.t0 = time.ticks_us()
            self.set_KD_parameter(self.K, self.B)
            self.first_time = False
            print("Stance State")
        else:  # do nothing (PVT will keep the previous config)
            pass

    def checkConditionForChangingState(self):
        # if self.robot.get_thigh_pos() >=0:
        t1 = time.ticks_us()
        # if self.robot.load_cell_Fz < self.Fz_tresh: #
        if time.ticks_diff(t1, self.t0) > 100000:
            return True
        else:
            return False

    def getNextState(self):
        self.first_time = True
        print("Stance to Early Swing")
        return "EarlySwing"


