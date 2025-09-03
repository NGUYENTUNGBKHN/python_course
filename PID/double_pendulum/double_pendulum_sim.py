import numpy as np
from numpy import pi, sin, cos
from functools import reduce
import matplotlib.pyplot as plt
from matplotlib import animation

# physical constants
g = 9.8
l1 = 0.8
l2 = 0.5
m1 = 1.0
m2 = 0.4

# simulation time
times = np.linspace(0, 20, 400)
dt = times[1] - times[0]

# initial conditions
a0 = pi / 3  # angle of the first pendulum
Y0 = .0  # angular velocity of the first pendulum
b0 = pi / 5  # angle of the second pendulum
Z0 = .0  # angular velocity of the second pendulum

initial_state = np.array([a0, Y0, b0, Z0])

def integrate_rk4(state, step, t, dt, dydx_func):
    k1 = derivate(state, step, t, dt)
    k2 = derivate([v + d * dt / 2 for v, d in zip(state, k1)], step, t, dt)
    k3 = derivate([v + d * dt / 2 for v, d in zip(state, k2)], step, t, dt)
    k4 = derivate([v + d * dt for v, d in zip(state, k3)], step, t, dt)
    return [v + (k1_ + 2 * k2_ + 2 * k3_ + k4_) * dt / 6 for v, k1_, k2_, k3_, k4_ in zip(state, k1, k2, k3, k4)]

def derivate(state, step, t, dt):
    ds = np.zeros_like(state)
    a = state[0]
    Y = state[1]
    b = state[2]
    Z = state[3]

    ds[0] = Y
    ds[1] = (- m2 * cos(a - b) * (l1 * Y * Y * sin(a - b) - g * sin(a - b)) - Z * Z * l2 * m2 * sin(a - b) - (
            m1 + m2) * g * sin(a)) / (l1 * (m1 + m2 * sin(a - b) * sin(a - b)))
    ds[2] = Z

    h = cos(a - b) / (m1 + m2)
    d = (m1 * l2 + m2 * l2 * sin(a - b) * sin(a - b)) / (m1 + m2)

    ds[3] = (h * (Z * Z * l2 * m2 * sin(a - b) + (m1 + m2) * g * sin(a)) + l1 * Y * Y * sin(a - b) - g * sin(b)) / d

    return ds


def solve(initial_state, times):
    dt = times[1] - times[0]
    states = [initial_state]
    for step, t in enumerate(times):
        states.append(integrate_rk4(states[-1], step, t, dt, derivate))
    return np.array(states)


solution = solve(initial_state, times)

alphas = solution[:, 0]
betas = solution[:, 2]

xs1 = l1 * sin(alphas)
ys1 = -l1 * cos(alphas)
xs2 = l2 * sin(betas) + xs1
ys2 = -l2 * cos(betas) + ys1

fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
ax.set_aspect('equal')
ax.grid()

line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)


def init():
    line.set_data([], [])
    time_text.set_text('')
    return line, time_text


def animate(i):
    thisx = [0, xs1[i], xs2[i]]
    thisy = [0, ys1[i], ys2[i]]

    line.set_data(thisx, thisy)
    time_text.set_text(time_template % (i * dt))
    return line, time_text


ani = animation.FuncAnimation(fig, animate, np.arange(1, len(solution)),
                              interval=25, blit=True, init_func=init)
plt.show()

