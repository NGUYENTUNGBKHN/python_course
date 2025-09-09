
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
        self.last_state = init_state
        self.integral_error = 0.0
    
    def get_ctrl(self, value, dt):
        error = value - self.target
        derivative = - (value - self.last_state) / dt
        integral = self.integral_error * error

        self.last_state = value
        self.integral_error += error
        result = (self.Kp * error + self.Ki * integral + self.Kd * derivative)
        return result

    def set_target(self, target) -> None:
        self.target = target

