import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from ode_solver.ode_solver import solve, integrate_rk4
from ode_solver.pid_ctrl import pid_ctrl
from dc.dc import DC

u_log = []

dc = DC()

velocity_pid = pid_ctrl(20.0, -0.1, 0.1, 0.0)
position_pid = pid_ctrl(2.0, 0.0, 0.0, 1.0)

def derivative(state, step, t, dt):
    w, theta = state
    w_target = position_pid.get_ctrl(theta, dt)
    velocity_pid.set_target(w_target)

    u = velocity_pid.get_ctrl(w, dt)
    u = u if u <= 12 else 12
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


