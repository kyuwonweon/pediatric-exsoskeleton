from lib.Robots.RobotInterface import Robot
from lib.ll_common.error_flag import print_error


class StateInterface:
    def __init__(self, robot:Robot):
        self.name = "StateInterface"
        self.robot = robot
        # defaul params values valid for all states
        self.q_des = 0.0
        self.qdot_des = 0.0
        self.dt = 1.0
        self.K = 0.5
        self.B = 0.1
        self.first_time = True

    # abstract methods to be overwritten in each state specific
    def handle(self):
        pass
    def checkConditionForChangingState(self):
        pass
    def getNextState(self):
        pass

    def set_KD_parameter(self, K: float, B: float) -> None:
        if not(self.robot.K == K) or not(self.robot.B == B):
            self.K = K
            self.B = B
            self.robot.set_KD_parameter(K, B)

    def get_params_state(self):
        return [self.q_des,
                self.qdot_des,
                self.dt, self.K, self.B]
    
    def restart_state(self):
        self.first_time= True

    def set_params_state(self, params):
        if len(params) != len(self.get_params_state()):
            raise Exception("Wrong number of parameters")
        print(self.name, params)
        self.q_des = params[0]
        self.qdot_des = params[1]
        self.dt = params[2]
        self.K = params[3]
        self.B = params[4]



