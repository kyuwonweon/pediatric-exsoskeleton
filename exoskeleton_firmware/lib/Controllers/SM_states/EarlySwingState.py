from lib.Controllers.SM_states.StateInterface import StateInterface
import time

class EarlySwingState(StateInterface):
    def __init__(self, robot):
        super().__init__(robot)
        self.name= "EarlySwing"
        self.q_des = 50.0
        self.dt = 2.0
        self.eps_pos = 10
        self.eps_vel = 5

        self.Fz_tresh=10


    def handle(self):
        if self.first_time:

            self.set_KD_parameter(self.K, self.B)
            self.robot.set_transition_point(self.q_des, self.qdot_des, self.dt)
            self.first_time = False
            print("Early Swing State")
        else: # do nothing (PVT will keep the previous config)
            pass

    def checkConditionForChangingState(self):
        if ((self.robot.get_position() >= self.q_des - self.eps_pos) and (self.robot.get_velocity() <= self.eps_vel)):
            return True
        else:
            return False

    def getNextState(self):
        self.first_time = True
        print("Early Swing 2 late swing")
        return "LateSwing"
