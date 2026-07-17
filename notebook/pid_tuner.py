# Proportional-Integral-Derivative (PID) Tuner for Stunt Car Inverted Pendulum Dynamics Model
# Author: Andrew D'Onofrio
# Date: July 17th, 2026

import numpy as np
import matplotlib.pyplot as plt

from pendulum_model import DEFAULT_PARAMS, simulate_closed_loop, PID


def linearize(params=DEFAULT_PARAMS):
    """Return (A, B) for the linearized plant: theta_ddot ~= A*theta - B*u"""
    l, w, g, m_car = params["l"], params["w"], params["g"], params["m_car"]
    I0 = (l**2 + w**2) / 12.0
    A = (l / 2.0) * g / I0
    B = (l / 2.0) / (m_car * I0)
    return A, B


def gains_from_p(p, params=DEFAULT_PARAMS):
    A, B = linearize(params)
    kd = 3.0 * p / B
    kp = (3.0 * p**2 + A) / B
    ki = p**3 / B
    return kp, ki, kd


def gains_from_wn_zeta(wn, zeta, params=DEFAULT_PARAMS):
    A, B = linearize(params)
    p_real = zeta * wn
    # Desired char. eq: (s + p_real)(s^2 + 2*zeta*wn*s + wn^2)
    s2_coeff = p_real + 2 * zeta * wn
    s1_coeff = wn**2 + 2 * zeta * wn * p_real
    s0_coeff = wn**2 * p_real

    kd = s2_coeff / B
    kp = (s1_coeff + A) / B
    ki = s0_coeff / B
    return kp, ki, kd


def verify(kp, ki, kd, theta0_deg=0.5, t_final=3.0, dt=0.002, params=DEFAULT_PARAMS):
    z0 = [0.0, 0.0, np.radians(theta0_deg), 0.0]
    pid = PID(kp=kp, ki=ki, kd=kd, setpoint=0.0, u_limit=2.0)
    sol = simulate_closed_loop(params, z0, t_final, dt, pid)
    return sol


def plot_comparison(p_values, theta0_deg=0.5):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for p in p_values:
        kp, ki, kd = gains_from_p(p)
        sol = verify(kp, ki, kd, theta0_deg=theta0_deg)
        label = f"p={p} (kp={kp:.1f}, ki={ki:.1f}, kd={kd:.2f})"
        ax1.plot(sol.t, np.degrees(sol.y[2]), label=label)
        ax2.plot(sol.t, sol.u, label=label)

    ax1.set_ylabel("theta (deg)")
    ax1.axhline(0, color="black", linewidth=0.6)
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3)

    ax2.set_ylabel("u (N)")
    ax2.set_xlabel("time (s)")
    ax2.axhline(2.0, color="red", linewidth=0.6, linestyle="--")
    ax2.axhline(-2.0, color="red", linewidth=0.6, linestyle="--")
    ax2.grid(alpha=0.3)

    fig.suptitle("Pole-Placement PID Gains -- Nonlinear Sim Verification")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    A, B = linearize()
    print(f"Linearized plant: theta_ddot = {A:.3f}*theta - {B:.3f}*u")
    print(f"Open-loop instability rate sqrt(A) = {np.sqrt(A):.3f} rad/s\n")

    print(f"{'p':>6} | {'kp':>8} {'ki':>8} {'kd':>8}")
    for p in [10, 15, 20, 25, 30]:
        kp, ki, kd = gains_from_p(p)
        print(f"{p:>6} | {kp:8.2f} {ki:8.2f} {kd:8.2f}")

    # Compare a few pole locations against the actual nonlinear simulator
    plot_comparison(p_values=[10, 20, 30], theta0_deg=0.5)