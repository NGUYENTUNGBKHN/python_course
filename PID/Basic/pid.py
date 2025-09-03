
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

        if (dt_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * dt_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard
            
            self.DTrem = 0.0
            if dt_time > 0:
                self.DTrem = dt_error / dt_time

            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTrem)

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain
    
    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

