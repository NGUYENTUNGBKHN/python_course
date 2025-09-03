
class PIDLongitudinalController():
    def __init__(
            self,
            p_gain: float = +1.2, # P Gain
            i_gain: float = +0.4, # I Gain
            d_gain: float = +0.1, # D Gain
            target_velocity: float = 3.0, # [m/s]
    ) -> None:
        """initialize pid controller for keeping constant velocity"""
        # pid control parameters
        self.K_p = p_gain
        self.K_i = i_gain
        self.K_d = d_gain
        self.target_vel = target_velocity
        self.pre_e = 0.0 # previous tracking error
        self.integrated_e = 0.0 # integrated tracking error

    def calc_control_input(self, observed_vel: float, delta_t: float) -> None:
        """calculate control input"""

        # calculate tracking error, its integral and derivative
        r = self.target_vel
        y = observed_vel
        e = r - y # tracking error to the traget velocity
        ie = self.integrated_e + (e + self.pre_e) * delta_t / 2.0 # integral of the tracking error
        de = (e - self.pre_e) / delta_t # derivative of the tracking error

        # calculate control input
        acc_cmd = self.K_p * e + self.K_i * ie + self.K_d * de

        # update previous tracking error
        self.pre_e = e

        return acc_cmd