# Laplace & S-Plane Analysis for Stunt Car Inverted Pendulum Dynamics Model
# Author: Andrew D'Onofrio
# Date: July 17th, 2026

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

from pendulum_model import DEFAULT_PARAMS
from pid_tuner import linearize, gains_from_p


def open_loop_tf(params=DEFAULT_PARAMS):
    """Return scipy TransferFunction for G(s) = -B / (s^2 - A)."""
    A, B = linearize(params)
    num = [-B]
    den = [1, 0, -A]
    return signal.TransferFunction(num, den), A, B

def open_loop_poles(params=DEFAULT_PARAMS):
    _, A, _ = open_loop_tf(params)
    return np.array([np.sqrt(A), -np.sqrt(A)])

def closed_loop_poles(kp, ki, kd, params=DEFAULT_PARAMS):
    """Roots of s^3 + B*kd*s^2 + (B*kp - A)*s + B*ki = 0."""
    A, B = linearize(params)
    coeffs = [1.0, B * kd, B * kp - A, B * ki]
    return np.roots(coeffs)

def plot_pole_zero(kp, ki, kd, params=DEFAULT_PARAMS, title_suffix=""):
    ol_poles = open_loop_poles(params)
    cl_poles = closed_loop_poles(kp, ki, kd, params)

    fig, ax = plt.subplots(figsize=(6, 6))

    ax.axhline(0, color="black", linewidth=0.6)
    ax.axvline(0, color="black", linewidth=0.6)

    ax.scatter(ol_poles.real, ol_poles.imag, marker="x", s=90, color="#C0402A",
               label="Open-loop poles", zorder=3)
    ax.scatter(cl_poles.real, cl_poles.imag, marker="x", s=90, color="#1A6B3C",
               label="Closed-loop poles (PID)", zorder=3)

    # Shade the stable (left-half-plane) region
    xlim = max(2.0, np.max(np.abs(np.concatenate([ol_poles.real, cl_poles.real]))) * 1.3)
    ylim = max(np.max(np.abs(np.concatenate([ol_poles.imag, cl_poles.imag]))) * 1.3 + 1, xlim * 0.5)
    ax.axvspan(-xlim, 0, color="#1A6B3C", alpha=0.05)

    ax.set_xlim(-xlim, xlim)
    ax.set_ylim(-ylim, ylim)
    ax.set_xlabel("Re(s)")
    ax.set_ylabel("Im(s)")
    ax.set_title(f"Pole-Zero Map (s-plane){title_suffix}")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), fontsize=9, ncol=2, frameon=False)
    fig.subplots_adjust(bottom=0.2)

    fig.tight_layout()
    plt.show()

if __name__ == "__main__":
    A, B = linearize()
    print(f"Open-loop plant: G(s) = -{B:.3f} / (s^2 - {A:.3f})")
    ol_p = open_loop_poles()
    print(f"Open-loop poles: {ol_p[0]:.3f}, {ol_p[1]:.3f}  <- unstable pole at +{ol_p[0]:.3f}\n")

    p = 20.0
    kp, ki, kd = gains_from_p(p)
    cl_p = closed_loop_poles(kp, ki, kd)
    print(f"PID gains for p={p}: kp={kp:.2f}, ki={ki:.2f}, kd={kd:.3f}")
    print("Closed-loop poles:")
    for root in cl_p:
        print(f"  {root:.3f}")

    plot_pole_zero(kp, ki, kd, title_suffix=f"  (p={p})")