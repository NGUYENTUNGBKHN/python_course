
import numpy as np

from numpy.linalg import matrix_rank
from control import ctrb, obsv
from ode_solver.lqr import lqr

class DC():
    """
        DC-motor simulation

        Equations of the motor:
            U = L * i' + R * i + k * w
            J * w' + B * w = k * i - T1
            -> w' = -w(B + k_e * k_t/R)/J + k_t * U / (R * J)
        where:
            U - Input voltage
            L - Rotor inductance
            R - Rotor resistance
            J - Rotor moment of inertia
            B - Rotor friction
            T1 - external load torque

            k_e - back EMF constant, k_e * w - back EMF
            k_t - torque constant
        T1 = r * m * a = r^2 * w' * m, wher r - shaft radius, m - load mass
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
    
    def get_dw(self, w, U):
        return -w * (self.B + self.k_e * self.k_t / self.R) / self.J + self.k_t * U / self.R / self.J


"""
    For DC
    situation 1:
        w' = (-B/J) * w + (k/J) * i + (-T1/J)
        i' = (-k/L) * w + (-R/L) * i + (1/L)*U
    situation 2:
        w' = -w(B + k_e * k_t/R)/J + k_t * U / (R * J)
        theta' = w
    Suppose : T1 = 0
"""
def get1_a_b(B, J, k, L, R):

    A = np.array([
        [-B/J, k/J],
        [-k/L, -R/L]
    ])

    B = np.array([[0, 1/L]])
    return [A, B]

def get2_a_b(B, J, k_e, k_t, L, R):

    A = np.array([
        [-(B + k_e*k_t/R)/J, 0.0],
        [1.0, 0.0]
    ])

    B = np.array([
        [k_t/(R * J)],
        [0]
    ])
    return [A, B]

def check_ctrb_obsv(A, B, C):
    ctrb_matrix = ctrb(A, B.T)
    obsv_matrix = obsv(A, C)

    print("Controllability matrix rank: %d" % matrix_rank(ctrb_matrix))
    print("Observability matrix rank: %d" % matrix_rank(obsv_matrix))


def get_lqr_gains(A, B, Q, R):
    K, _, _ = lqr(A, B, Q, R)
    return K


if __name__ == "__main__":

    dc = DC()
    A, B = get2_a_b(dc.B, dc.J, dc.k_t, dc.k_e, dc.L, dc.R)
    # check_ctrb_obsv(A, B, C=np.array([[1, 0, 1, 0]]))
    print(A, B)
    Q = np.array([
        [10, .0],
        [.0, 1.],
    ])
    R = np.array([[10.]])
    print(get_lqr_gains(A, B, Q, R))


