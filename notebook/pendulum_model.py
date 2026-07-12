"""
Reusable simulation core for the stunt-car inverted-pendulum model.

Deliberately contains only physics -- no GUI, no plotting -- so it can be
imported by more than one front end without duplicating the dynamics:
pendulum_gui.py (a Tkinter desktop window) imports this directly, and a
Jupyter/ipywidgets notebook interface (which, unlike Tkinter, *does* render
inside Codespaces) could import the exact same functions later without any
changes here.

State z = [x, x_dot, theta, theta_dot], input u = F_traction. See the
project README (Sections 1-3) for the derivation.

Drag is modeled as quadratic: F_Drag = Cd * x_dot * |x_dot| (the |x_dot|
keeps it always opposing motion, unlike a plain x_dot**2 which would push
the wrong way when x_dot < 0). Tracing this through the mass-matrix
inversion shows the drag term cancels out of the theta_ddot equation
entirely regardless of its functional form, so only x_ddot changes here
compared to the earlier linear-drag version. Linearizing x_dot*|x_dot|
about x_dot=0 gives a derivative of zero, so the linearized model (see
Section 3 / A matrix) has no damping term at all -- quadratic drag is
negligible for small perturbations near equilibrium, which is a real
property of the physics, not an approximation error.
"""

import numpy as np
from scipy.integrate import solve_ivp

DEFAULT_PARAMS = {
    "m_car": 0.38955,  # kg
    "l": 0.175,        # m
    "w": 0.140,         # m
    "Cd": 0.05,         # N*s^2/m^2 (placeholder -- calibrate against hardware)
    "g": 9.81,          # m/s^2
}


def J_of(l, w):
    return (1.0 / 3.0) * l**2 + (1.0 / 12.0) * w**2


def D_of(theta, l, w):
    return (1.0 / 12.0) * (l**2 + w**2) + (l**2 / 4.0) * np.sin(theta) ** 2


def dynamics(t, z, params, u_fn):
    x, x_dot, theta, theta_dot = z
    m_car, l, w, Cd, g = params["m_car"], params["l"], params["w"], params["Cd"], params["g"]
    u = u_fn(t)

    J = J_of(l, w)
    D = D_of(theta, l, w)
    s, c = np.sin(theta), np.cos(theta)
    drag = Cd * x_dot * abs(x_dot)

    x_ddot = (
        J * u
        + drag * ((l**2 / 4.0) * c**2 - J)
        + J * m_car * (l / 2.0) * s * theta_dot**2
        - (l**2 / 4.0) * m_car * g * s * c
    ) / (m_car * D)

    theta_ddot = (l / 2.0) / D * (g * s - u * c / m_car - (l / 2.0) * theta_dot**2 * s * c)

    return [x_dot, x_ddot, theta_dot, theta_ddot]


def simulate(params, z0, t_final, dt, u_fn):
    """Integrate the nonlinear ODE. z0 = [x0, xdot0, theta0_rad, thetadot0_rad]."""
    t_eval = np.arange(0.0, t_final + dt, dt)
    sol = solve_ivp(
        dynamics,
        (0.0, t_final),
        z0,
        args=(params, u_fn),
        t_eval=t_eval,
        method="RK45",
        max_step=dt,
    )
    return sol