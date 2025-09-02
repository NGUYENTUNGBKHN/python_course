
import numpy as np
from numpy import sin, cos, pi

import matplotlib.pyplot as plt
from matplotlib import animation
from pid_controller import PIDController

r = 0.25        # radius of wheel
l = 1.0         # length of pendulum
M = 0.25        # 
m = 0.3
g = 9.8

I = 0.5 * M * r 

# Friction coefficient due to rotation of the body
b1 = 0.01

# Friction coefficient due to rotation of the wheel
b2 = 0.01

# Measurement distortion
# - low-pass filter
# - delay
# - discrete measurement

th_pid = PIDController(k_p=40.0, k_d=20.0, k_i=0.0, target=0.0)

velocity_pid = PIDController(k_p=0.0085, k_d=0.0, k_i=0.001, target=0.0)

position_pid = PIDController(k_p=0.07, k_d=0.07, k_i=0.0, target=0.0)

u_history = []

def limit(v, lim):
    if v > lim:
        return lim
    elif v < -lim:
        return -lim
    else:
        return v


def get_ref(t):
    if t < 3.0:
        return np.array([[0, 0, t, 0]])
    else:
        x = 0.5
        return np.array([[0, 0, x / r, 0]])


def low_pass_filter(alpha, init):
    y = init

    def _inner(x):
        nonlocal y
        y = alpha * x + (1 - alpha) * y
        return y

    return _inner


def time_delay(n):
    buf = [0] * n

    def _inner(x):
        nonlocal buf
        val = buf[0]
        buf = buf[1:] + [x]
        return val

    return _inner


def discrete_value(n):
    y = 0
    i = n

    def _inner(x):
        nonlocal y, i
        i -= 1
        if i == 0:
            y = x
            i = n
        return y

    return _inner


filtered_measurement = low_pass_filter(1.0, 0.0)
delayed_measurement = time_delay(25)
discrete_measurement = discrete_value(25)

def integrate_rk4(state, step, t, dt, dydx_func):
    k1 = derivate(state, step, t, dt)
    k2 = derivate([v + d * dt / 2 for v, d in zip(state, k1)], step, t, dt)
    k3 = derivate([v + d * dt / 2 for v, d in zip(state, k2)], step, t, dt)
    k4 = derivate([v + d * dt for v, d in zip(state, k3)], step, t, dt)
    return [v + (k1_ + 2 * k2_ + 2 * k3_ + k4_) * dt / 6 for v, k1_, k2_, k3_, k4_ in zip(state, k1, k2, k3, k4)]

def derivate(state, step, t, dt):
    dth, th, dphi, phi = state

    # velocity_target = position_pid.get_control(phi, dt)
    # velocity_pid.set_target(velocity_target)
    #
    th_target = velocity_pid.get_control(dphi, dt)
    th_pid.set_target(th_target)
    # th_pid.set_target(velocity_target)

    # measured_th = discrete_measurement(th)
    measured_th = th
    u = -th_pid.get_control(measured_th, dt)
    u = limit(u, 20)
    u_history.append(u)

    s = sin(th)
    c = cos(th)

    _dphi = (m * r * (l * dth ** 2 * s + b1 * dth * c - g * s * c) - b2 * dphi + u) / (I + m * r ** 2 * s ** 2)
    _dth = (g * s - r * _dphi * c - b1 * dth) / l

    return [_dth, dth, _dphi, dphi]

def solve(initial_state, times):
    dt = times[1] - times[0]
    states = [initial_state]
    for step, t in enumerate(times):
        states.append(integrate_rk4(states[-1], step, t, dt, derivate))
    return np.array(states)




if __name__ == "__main__":
    times = np.linspace(0, 10, 500)
    solution = solve([0.0, pi/3, .0, .0], times)
    theta = solution[:, 1]
    phi = solution[:, 3]

    wheel_x = phi * r

    spot_r = 0.7 * r
    wheel_spot_x = wheel_x + spot_r * cos(phi - pi / 2)
    wheel_spot_y = r - spot_r * sin(phi - pi / 2)

    mass_x = wheel_x + l * cos(theta - pi / 2)
    mass_y = r - l * sin(theta - pi / 2)


    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-10, 10), ylim=(-5, 5))
    ax.set_aspect('equal')
    ax.grid()

    line, = ax.plot([], [], 'k-', lw=2)
    wheel = plt.Circle((0.0, r), r, color='black', fill=False, lw=2)
    wheel_spot = plt.Circle((0.0, spot_r), 0.02, color='red')
    mass = plt.Circle((0.0, 0.0), 0.1, color='black')

    # wheel.set_center((wheel_x[1], r))
    # wheel_spot.set_center((wheel_spot_x[1], wheel_spot_y[1]))
    # mass.set_center((mass_x[1], mass_y[1]))
    # line.set_data([wheel_x[1], mass_x[1]], [r, mass_y[1]])
    # patches = [line, ax.add_patch(wheel), ax.add_patch(wheel_spot), ax.add_patch(mass)]

    def init():
        return []


    def animate(i):
        wheel.set_center((wheel_x[i], r))
        wheel_spot.set_center((wheel_spot_x[i], wheel_spot_y[i]))
        mass.set_center((mass_x[i], mass_y[i]))
        line.set_data([wheel_x[i], mass_x[i]], [r, mass_y[i]])
        patches = [line, ax.add_patch(wheel), ax.add_patch(wheel_spot), ax.add_patch(mass)]
        # ax.set_xlim(mass_x[i] - 1, mass_x[i] + 1)
        # ax.set_ylim(mass_y[i] - 1, mass_y[i] + 1)
        return patches


    ani = animation.FuncAnimation(fig, animate, np.arange(1, len(solution)),
                                  interval=25, blit=True, init_func=init)

    plt.show()

