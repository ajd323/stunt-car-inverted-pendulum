# Physics and Simulation Back-End for Stunt Car Inverted Pendulum Dynamics Model
# Author: Andrew D'Onofrio
# Date: July 11th, 2026

# Imports for mathematical computations and plotting
import numpy as np
from scipy.integrate import solve_ivp
from types import SimpleNamespace

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

class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0, u_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.u_limit = u_limit
        self.integral = 0.0
 
    def reset(self):
        self.integral = 0.0
 
    def compute(self, dt, theta, theta_dot):
        error = theta - self.setpoint
        u_unsat = self.kp * error + self.ki * self.integral + self.kd * theta_dot
 
        if self.u_limit is not None:
            u = float(np.clip(u_unsat, -self.u_limit, self.u_limit))
            saturated = u != u_unsat
        else:
            u = u_unsat
            saturated = False
        if not saturated or (u_unsat * error < 0):
            self.integral += error * dt
        return u
 
def _rk4_step(z, t, dt, params, u):
    k1 = np.array(stunt_car_dynamics(t, z, params, lambda _t: u))
    k2 = np.array(stunt_car_dynamics(t + dt / 2.0, z + dt / 2.0 * k1, params, lambda _t: u))
    k3 = np.array(stunt_car_dynamics(t + dt / 2.0, z + dt / 2.0 * k2, params, lambda _t: u))
    k4 = np.array(stunt_car_dynamics(t + dt, z + dt * k3, params, lambda _t: u))
    return z + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
 
 
def simulate_closed_loop(params, z0, t_final, dt, controller, stop_on_fall=True, hold_after_fall=True):
    controller.reset()
 
    n_steps = int(round(t_final / dt)) + 1
    t_arr = np.linspace(0.0, n_steps * dt, n_steps, endpoint=False)
    z_arr = np.zeros((4, n_steps))
    u_arr = np.zeros(n_steps)
 
    z = np.array(z0, dtype=float)
    fell_at = None
 
    for i in range(n_steps):
        t = t_arr[i]
        theta, theta_dot = z[2], z[3]
        u = controller.compute(dt, theta, theta_dot)
 
        z_arr[:, i] = z
        u_arr[i] = u
 
        if stop_on_fall and np.cos(theta) <= 0.0 and fell_at is None:
            fell_at = t
            if not hold_after_fall:
                t_arr = t_arr[: i + 1]
                z_arr = z_arr[:, : i + 1]
                u_arr = u_arr[: i + 1]
                break
            # hold the fallen state constant for the remaining samples
            z_arr[:, i:] = z.reshape(-1, 1)
            u_arr[i:] = u
            break
 
        z = _rk4_step(z, t, dt, params, u)
 
    return SimpleNamespace(t=t_arr, y=z_arr, u=u_arr, fell_at=fell_at, success=True, message="")