
class DC():
    """
        DC-motor simulation

        Equations of the motor:
            U = L * i' + R * i + k * w
            J * w' + B * w = k * i - T1
        where:
            U - Input voltage
            L - Rotor inductance
            R - Rotor resistance
            J - Rotor mementum of inertia
            B - Rotor friction
            T1 - external load torque

            k_e - back EMF constant, k_e * w - back EMF
            k_t - torque constant
        T1 = r * m * a = r^2 * w' * m, wher r - shaft redius, m - load mass
    """
    def __init__(
        self,
        L: float = 0.01,
        R: float = 4.0,
        J: float = 0.3,
        B: float = 1e-2,
        k_e: float = 1.0,
        k_t: float = 1.0
    ):
        self.L = L
        self.R = R
        self.J = J
        self.B = B
        self.k_e = k_e
        self.k_t = k_t
    
    def get_dw(self, w, u):
        return - w * (self.B + self.k_e)



