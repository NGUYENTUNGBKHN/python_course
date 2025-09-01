import numpy as np
from numpy import sin, cos, pi

import matplotlib.pyplot as plt
from matplotlib import animation

def solve(initial_state, times, integrate_func, derivative_func):
    dt = times[1] - times[0]
    states = [initial_state]
    for step, t in enumerate(times):
        states.append(integrate_func(states[-1], step, t, dt, derivative_func))
    return np.array(states)

def integrate_rk4(state, step, t, dt, dydx_func):
    k1 = dydx_func(state, step, t, dt)
    k2 = dydx_func([v + d * dt / 2 for v, d in zip(state, k1)], step, t, dt)
    k3 = dydx_func([v + d * dt / 2 for v, d in zip(state, k2)], step, t, dt)
    k4 = dydx_func([v + d * dt for v, d in zip(state, k3)], step, t, dt)
    return [v + (k1_ + 2 * k2_ + 2 * k3_ + k4_) * dt / 6 for v, k1_, k2_, k3_, k4_ in zip(state, k1, k2, k3, k4)]

r = 0.25
l = 1.0
M = 0.25
m = 0.3
g = 9.8

I = 0.5 * M * r

def derivate(state, step, t, dt):
    dth, th, dphi, phi = state

    _dphi = (m * l * r * dth ** 2 * sin(th) - m * g * r * sin(th) * cos(th)) / (m * r ** 2 * sin(th) ** 2 + I)
    _dth = (g * sin(th) - r * _dphi * cos(th)) / l

    return [_dth, dth, _dphi, dphi]


if __name__ == "__main__":
    times = np.linspace(0, 10, 500)
    solution = solve([0.0, pi / 12, .0, .0], times, integrate_rk4, derivate)
    theta = solution[:, 1]
    phi = solution[:, 3]

    wheel_x = phi * r

    spot_r = 0.7 * r
    wheel_spot_x = wheel_x + spot_r * cos(phi - pi / 2)
    wheel_spot_y = r - spot_r * sin(phi - pi / 2)

    mass_x = wheel_x + l * cos(theta - pi / 2)
    mass_y = r - l * sin(theta - pi / 2)

    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1.5, 1.5), ylim=(-1.5, 1.5))
    ax.set_aspect('equal')
    ax.grid()

    line, = ax.plot([], [], 'k-', lw=2)
    wheel = plt.Circle((0.0, r), r, color='black', fill=False, lw=2)
    wheel_spot = plt.Circle((0.0, spot_r), 0.02, color='red')
    mass = plt.Circle((0.0, 0.0), 0.1, color='black')


    def init():
        return []


    def animate(i):
        wheel.set_center((wheel_x[i], r))
        wheel_spot.set_center((wheel_spot_x[i], wheel_spot_y[i]))
        mass.set_center((mass_x[i], mass_y[i]))
        line.set_data([wheel_x[i], mass_x[i]], [r, mass_y[i]])
        patches = [line, ax.add_patch(wheel), ax.add_patch(wheel_spot), ax.add_patch(mass)]
        return patches


    ani = animation.FuncAnimation(fig, animate, np.arange(1, len(solution)),
                                  interval=25, blit=True, init_func=init)

    plt.show()


   
