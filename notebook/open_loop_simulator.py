import argparse
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp


def build_params(args):
    return {
        "m_car": args.mass,
        "l": args.length,
        "w": args.width,
        "b": args.damping,
        "g": args.gravity,
    }


def J_of(l, w):
    return (1.0 / 3.0) * l**2 + (1.0 / 12.0) * w**2


def D_of(theta, l, w):
    return (1.0 / 12.0) * (l**2 + w**2) + (l**2 / 4.0) * np.sin(theta) ** 2


def dynamics(t, z, params, u_fn):
    x, x_dot, theta, theta_dot = z
    m_car, l, w, b, g = params["m_car"], params["l"], params["w"], params["b"], params["g"]
    u = u_fn(t)

    J = J_of(l, w)
    D = D_of(theta, l, w)
    s, c = np.sin(theta), np.cos(theta)

    x_ddot = (
        J * u
        + b * x_dot * ((l**2 / 4.0) * c**2 - J)
        + J * m_car * (l / 2.0) * s * theta_dot**2
        - (l**2 / 4.0) * m_car * g * s * c
    ) / (m_car * D)

    theta_ddot = (l / 2.0) / D * (g * s - u * c / m_car - (l / 2.0) * theta_dot**2 * s * c)

    return [x_dot, x_ddot, theta_dot, theta_ddot]


def simulate(params, z0, t_final, dt, u_fn):
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


def plot_response(sol, params, out_path):
    t = sol.t
    x, x_dot, theta, theta_dot = sol.y
    theta_deg = np.degrees(theta)
    theta_dot_deg = np.degrees(theta_dot)

    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    fig.suptitle(
        f"Open-loop response  (m_car={params['m_car']*1000:.1f} g, "
        f"l={params['l']*1000:.0f} mm, w={params['w']*1000:.0f} mm)"
    )

    axes[0, 0].plot(t, x * 1000, color="#1A2033")
    axes[0, 0].set_title("Position, x(t)")
    axes[0, 0].set_ylabel("x (mm)")
    axes[0, 0].grid(alpha=0.3)

    axes[0, 1].plot(t, x_dot * 1000, color="#5A5F6E")
    axes[0, 1].set_title("Velocity, ẋ(t)")
    axes[0, 1].set_ylabel("ẋ (mm/s)")
    axes[0, 1].grid(alpha=0.3)

    axes[1, 0].plot(t, theta_deg, color="#C0402A")
    axes[1, 0].axhline(0, color="black", linewidth=0.6)
    axes[1, 0].set_title("Angle, θ(t)")
    axes[1, 0].set_ylabel("θ (deg)")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 0].grid(alpha=0.3)

    axes[1, 1].plot(t, theta_dot_deg, color="#D85A30")
    axes[1, 1].set_title("Angular velocity, θ̇(t)")
    axes[1, 1].set_ylabel("θ̇ (deg/s)")
    axes[1, 1].set_xlabel("time (s)")
    axes[1, 1].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description="Open-loop stunt-car pendulum simulator")
    p.add_argument("--mass", type=float, default=0.38955, help="m_car [kg] (default: 0.38955)")
    p.add_argument("--length", type=float, default=0.175, help="l [m] (default: 0.175)")
    p.add_argument("--width", type=float, default=0.140, help="w [m] (default: 0.140)")
    p.add_argument("--damping", type=float, default=0.01, help="b [N*s/m] (default: 0.01, placeholder)")
    p.add_argument("--gravity", type=float, default=9.81, help="g [m/s^2]")
    p.add_argument("--x0", type=float, default=0.0, help="initial position [m]")
    p.add_argument("--xdot0", type=float, default=0.0, help="initial velocity [m/s]")
    p.add_argument("--theta0", type=float, default=0.5, help="initial angle [deg]")
    p.add_argument("--thetadot0", type=float, default=0.0, help="initial angular velocity [deg/s]")
    p.add_argument("--u", type=float, default=0.0, help="constant traction force F_Traction [N] (default: 0, open loop)")
    p.add_argument("--t-final", type=float, default=3.0, help="simulation duration [s]")
    p.add_argument("--dt", type=float, default=0.002, help="time step [s]")
    p.add_argument("--output", type=str, default="pendulum_response.png", help="output plot filename")
    args = p.parse_args()

    params = build_params(args)
    z0 = [args.x0, args.xdot0, np.radians(args.theta0), np.radians(args.thetadot0)]
    u_fn = lambda t: args.u

    sol = simulate(params, z0, args.t_final, args.dt, u_fn)
    if not sol.success:
        print("Integration failed:", sol.message)
        return

    plot_response(sol, params, args.output)

    theta_deg = np.degrees(sol.y[2])
    print(f"Simulated {args.t_final:.2f} s with theta0={args.theta0} deg, u={args.u} N (constant).")
    print(f"Final theta: {theta_deg[-1]:.2f} deg, peak |theta|: {np.max(np.abs(theta_deg)):.2f} deg")
    print(f"Plot saved to: {args.output}")


if __name__ == "__main__":
    main()