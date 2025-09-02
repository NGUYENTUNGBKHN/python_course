
import time

class PIDController:
    def __init__(self, P=0.02, I=0.0, D=0.0, current_time=None):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.0
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTrem = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0
        self.output = 0.0
    
    def update(self, feedback_value, current_time=None):

        error = self.SetPoint - feedback_value
        self.current_time = current_time if current_time is not None else time.time()
        dt_time = self.current_time - self.last_time
        dt_error = error - self.last_error

