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


def fall_event(t, z, params, u_fn):
    """
    Zero-crossing event for scipy's solve_ivp: fires when the body's center
    of mass reaches ground level. y_COM = (l/2)*cos(theta) (see Section 1),
    so y_COM = 0 exactly when theta = +/-90 degrees -- that's "altitude
    zero," i.e. the car has fallen flat, regardless of which direction it
    tipped. cos(theta) is used directly rather than y_COM itself since l
    cancels out of the zero-crossing condition anyway.
    """
    return np.cos(z[2])


fall_event.terminal = True
fall_event.direction = -1  # only trigger while falling away from upright, not recovering


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


def simulate(params, z0, t_final, dt, u_fn, stop_on_fall=True, hold_after_fall=True):
    """
    Integrate the nonlinear ODE. z0 = [x0, xdot0, theta0_rad, thetadot0_rad].

    If stop_on_fall is True, integration halts the first time |theta|
    reaches 90 degrees (the car's CM has hit the ground -- see fall_event).
    sol.fell_at is set to that time, or None if it never fell.

    If it did fall and hold_after_fall is True (default), the returned
    arrays are padded out to t_final by holding the fallen state constant,
    so plots show a flat line after the fall instead of just stopping short
    -- useful for comparing runs on the same time axis. Set
    hold_after_fall=False to get a plot that simply ends at the fall time
    instead.
    """
    t_eval = np.arange(0.0, t_final + dt, dt)
    events = [fall_event] if stop_on_fall else None
    sol = solve_ivp(
        dynamics,
        (0.0, t_final),
        z0,
        args=(params, u_fn),
        t_eval=t_eval,
        method="RK45",
        max_step=dt,
        events=events,
    )

    fell_at = None
    if stop_on_fall and len(sol.t_events[0]) > 0:
        fell_at = float(sol.t_events[0][0])

    if fell_at is not None and hold_after_fall:
        last_t, last_y = sol.t[-1], sol.y[:, -1]
        remaining_t = t_eval[t_eval > last_t]
        if len(remaining_t) > 0:
            pad_y = np.tile(last_y.reshape(-1, 1), (1, len(remaining_t)))
            sol.t = np.concatenate([sol.t, remaining_t])
            sol.y = np.concatenate([sol.y, pad_y], axis=1)

    sol.fell_at = fell_at
    return sol