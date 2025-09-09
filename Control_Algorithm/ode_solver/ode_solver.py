import numpy as np
"""
    ODE is ordinary differential equation
"""



"""
    Euler Method

    parameter : state
                step
                t
                dt
                derivative
    formula :
        yn+1 = yn + h.f(xn,yn):

"""
def integrate_euler(state, step, t, dt, derivative_func):
    k1 = derivative_func(state, step, t, dt)
    return [v + k1_ * dt for v, k1_ in zip(state, k1)]

"""
    Runge Kutta

    formula :
        dy/dx = f(x,y)

        The iterative formula:
        yn+1 = yn + 1/6(k1 + 2k2 + 2k3 + k4)h

        where h is the step size (xn+1 = xn + h)
        k1 = f(xn, yn)
        k2 = f(xn + h/2, yn + h/2*k1)
        k3 = f(xn + h/2, yn + h/2*k2)
        k4 = f(xn + h/2, yn + h*k3)
"""
def integrate_rk4(state, step, t, dt, derivative_func):

    k1 = derivative_func(state, step, t, dt)
    k2 = derivative_func([v + k1_ * dt/2 for v, k1_ in zip(state, k1)], step, t + dt / 2, dt)
    k3 = derivative_func([v + k2_ * dt/2 for v, k2_ in zip(state, k2)], step, t + dt / 2 , dt)
    k4 = derivative_func([v + k3_ * dt for v, k3_ in zip(state, k3)], step, t + dt /2, dt)

    return [v + ( k1_ + 2*k2_ + 2*k3_ + k4_)*dt/6 for v, k1_, k2_, k3_, k4_ in zip(state, k1, k2, k3, k4)]

def solve(initial_state, times, integrate_func, derivative_func)
    dt = times[1] - times[0]
    states = initial_state
    for step, t in enumerate(times):
        states.append(integrate_func(states[-1], step, t, dt, derivative_func))
    return np.array(states)
