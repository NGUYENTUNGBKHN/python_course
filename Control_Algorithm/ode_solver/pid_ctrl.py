
class pid_ctrl():
    def __init__(
            self,
            Kp,
            Ki,
            Kd,
            target,
            init_state: float = 0.0
        ) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = target
        self.lastvalue = init_state
        self.integral_error = 0.0
    
    def get_ctrl(self, value, dt):
        error = self.target - value

        derivative = -(value - self.lastvalue) / dt

        self.lastvalue = value
        self.integral_error += error * dt
        return self.Kp * error + self.Kd * derivative + self.Ki * self.integral_error

    def set_target(self, target) -> None:
        self.target = target

