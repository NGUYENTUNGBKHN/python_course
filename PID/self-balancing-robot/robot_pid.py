
import numpy as np
from numpy import sin, cos, pi

import matplotlib.pyplot as plt
from matplotlib import animation

r = 0.25        # radius of wheel
l = 1.0         # length of pendulum
M = 0.25        # 
m = 0.3
g = 9.8

I = 0.5 * M * r 

def integrate_rk4(state, step, t, dt, dydx_func):
    k1 = derivate(state, step, t, dt)
    k2 = derivate([v + d * dt / 2 for v, d in zip(state, k1)], step, t, dt)
    k3 = derivate([v + d * dt / 2 for v, d in zip(state, k2)], step, t, dt)
    k4 = derivate([v + d * dt for v, d in zip(state, k3)], step, t, dt)
    return [v + (k1_ + 2 * k2_ + 2 * k3_ + k4_) * dt / 6 for v, k1_, k2_, k3_, k4_ in zip(state, k1, k2, k3, k4)]

def derivate(state, step, t, dt):
    dth, th, dphi, phi = state
    _dphi = (m * l * r * dth ** 2 * sin(th) - m * g * r * sin(th) * cos(th)) / (m * r ** 2 * sin(th) ** 2 + I)
    _dth = (g * sin(th) - r * _dphi * cos(th)) / l
    return [_dth, dth, _dphi, dphi]

def solve(initial_state, times):
    dt = times[1] - times[0]
    states = [initial_state]
    for step, t in enumerate(times):
        states.append(integrate_rk4(states[-1], step, t, dt, derivate))
    return np.array(states)




if __name__ == "__main__":
    times = np.linspace(0, 10, 500)
    solution = solve([0.0, pi / 12, .0, .0], times)

    spot_r = 0.7 * r

    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1.5, 1.5), ylim=(-1.5, 1.5))
    ax.set_aspect('equal')
    ax.grid()

    line, = ax.plot([], [], 'k-', lw=2)
    wheel = plt.Circle((0.0, r), r, color='black', fill=False, lw=2)
    wheel_spot = plt.Circle((0.0, spot_r), 0.02, color='red')
    mass = plt.Circle((0.0, 0.0), 0.1, color='black')

    plt.show()

