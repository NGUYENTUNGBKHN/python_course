import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from ode_solver.ode_solver import solve, integrate_rk4
from ode_solver.lqr import lqr
from dc.dc import DC, get1_a_b, get2_a_b, get_lqr_gains

u_log = []

dc = DC()

A, B = get2_a_b(dc.B, dc.J, dc.k_t, dc.k_e, dc.L, dc.R)
Q = np.array([
        [0.1, .0], # w
        [.0, 1],   # theta
    ])
R = np.array([[0.01]])
K = get_lqr_gains(A, B, Q, R)

def get_ref(t):
    return np.array([[0.0, 1.0]])

def derivative(state, step, t, dt):
    w, theta = state

    _state = np.array([[w,theta]])
    u = (-K @ (_state - get_ref(t)).T)[0, 0]
    u = np.clip(u, -12, 12)
    u_log.append(u)
    return [dc.get_dw(w, u), w]

if __name__ == "__main__":
    times = np.linspace(0, 3.0, 1000)

    solution = []
    
    solution = solve([0, 0], times, integrate_rk4, derivative)

    w_line, = plt.plot(times, solution[:-1, 0], label="ω")
    th_line, = plt.plot(times, solution[:-1, 1], label="Θ")
    u_line, = plt.plot(times, u_log[::4], label="U")
    plt.legend([w_line, th_line, u_line], ['ω', 'Θ', 'U'])
    plt.grid(True)
    plt.show()


