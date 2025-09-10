

"""
    PID - Proportional, integral, derivation

    
"""
class pid_ctrl():
    def __init__(self, Kp, Ki, Kd, target, init_state = 0.0) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_state = init_state
        self.target = target
        self.integrate_error = 0

    def get_ctrl(self, value, dt):
        error = self.target - value

        derivative = -(value - self.last_state) /dt
        self.integrate_error += error * dt
        self.last_state = value

        return (self.Kp * error + self.Ki * self.integrate_error + self.Kd * derivative)
    
    def set_target(self, target):
        self.target = target




