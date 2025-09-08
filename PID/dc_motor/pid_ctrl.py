# import numpy as np


class PID_CTRL():
    def __init__(
        self,
        Kp          : float = 0.0,
        Ki          : float = 0.0,
        Kd          : float = 0.0,
        target      : float = 0.0,
        init_state  : float = 0.0
    ) -> None:

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.target = target
        self.last_state = init_state


    def get_control(
        self,
        value,
        dt
    ):
        error = value - self.target
        derivative = (value - self.last_state) /dt
        self.last_state = value
        self.integral += error * self

        result = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        return result

    def get_target(self):
        return self.target
        




