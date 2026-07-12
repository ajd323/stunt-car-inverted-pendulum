# Physics and Simulation Back-End for Stunt Car Inverted Pendulum Dynamics Model
# Author: Andrew D'Onofrio
# Date: July 11th, 2026

# Imports for mathematical computations and plotting
import numpy as np
from scipy.integrate import solve_ivp

# Dictionaries
DEFAULT_PARAMS = {
    "m_car": 0.38955, # Mass, kg
    "l": 0.175, # Length, m
    "w": 0.140, # Width, m
    "Cd": 1.12, # Drag Coefficient, N*s^2/m^2
    "g": 9.81, # Gravity, m/s^2
}

# Functions
def fall_event(t, z, params, u_fn):
    return np.cos(z[2])
# Event flags for car fallen
fall_event.terminal = True
fall_event.direction = -1

def stunt_car_dynamics(t, z, params, u_fn):
    # Redfining and Simplifying Variables
    x, x_dot, theta, theta_dot = z
    m_car, l, w, Cd, g = params["m_car"], params["l"], params["w"], params["Cd"], params["g"]
    u = u_fn(t)
    s, c = np.sin(theta), np.cos(theta)
    drag = Cd * x_dot * abs(x_dot)

    # Deriving Second Order ODEs for x_ddot and theta_ddot
    x_ddot = (
        ((1.0 / 3.0) * l**2 + (1.0 / 12.0) * w**2) * u
        + drag * ((l**2 / 4.0) * c**2 - ((1.0 / 3.0) * l**2 + (1.0 / 12.0) * w**2))
        + ((1.0 / 3.0) * l**2 + (1.0 / 12.0) * w**2) * m_car * (l / 2.0) * s * theta_dot**2
        - (l**2 / 4.0) * m_car * g * s * c
    ) / (m_car * ((1.0 / 12.0) * (l**2 + w**2) + (l**2 / 4.0) * np.sin(theta) ** 2))
    theta_ddot = ((l / 2.0) / ((1.0 / 12.0) * (l**2 + w**2) + (l**2 / 4.0) * np.sin(theta) ** 2)
        * (g * s - u * c / m_car - (l / 2.0) * theta_dot**2 * s * c))

    return [x_dot, x_ddot, theta_dot, theta_ddot]

def simulate(params, z0, t_final, dt, u_fn, stop_on_fall = True, hold_after_fall = True):
    t_eval = np.arange(0.0, t_final + dt, dt)
    events = [fall_event] if stop_on_fall else None

    # SciPy Numerical ODE Solver for Time-Sepcified Dynamics (with Initial Value Problem (IVP))
    solution = solve_ivp(stunt_car_dynamics, (0.0, t_final), z0, t_eval=t_eval,
        args=(params, u_fn), max_step=dt, events=events)

    # Fall Detection and Handling
    fell_at = None
    if stop_on_fall and len(solution.t_events[0]) > 0:
        fell_at = float(solution.t_events[0][0])
    if fell_at is not None and hold_after_fall:
        last_t, last_y = solution.t[-1], solution.y[:, -1]
        remaining_t = t_eval[t_eval > last_t]
        if len(remaining_t) > 0:
            pad_y = np.tile(last_y.reshape(-1, 1), (1, len(remaining_t)))
            solution.t = np.concatenate([solution.t, remaining_t])
            solution.y = np.concatenate([solution.y, pad_y], axis=1)
    solution.fell_at = fell_at

    return solution