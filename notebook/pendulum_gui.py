# User Interface (UI) for Stunt Car Inverted Pendulum Dynamics Model
# Author: Andrew D'Onofrio
# Date: July 17th, 2026

# Imports from Tinker Toolkit
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

# Imports for mathematical computations and plotting
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

# Imports from mathmetical part of pendulum model
from pendulum_model import DEFAULT_PARAMS, simulate, simulate_closed_loop, PID
from pid_tuner import linearize, gains_from_p

# Import Threading
import threading

# User Interface Parameters
FIELD_GROUPS = [
    ("Stunt Car Geometric / Physical Parameters", [
        ("mass", "Mass [kg]", DEFAULT_PARAMS["m_car"]),
        ("length", "Length [m]", DEFAULT_PARAMS["l"]),
        ("width", "Width [m]", DEFAULT_PARAMS["w"]),
        ("damping", "Drag Coefficient [N*s^2/m^2]", DEFAULT_PARAMS["Cd"]),
    ]),
    ("Initial Conditions", [
        ("x0", "Initial Position [m]", 0.0),
        ("xdot0", "Initial Velocity [m/s]", 0.0),
        ("theta0", "Initial Angle [deg]", 0.5),
        ("thetadot0", "Initial Angular Velocity [deg/s]", 0.0),
    ]),
    ("Simulation Settings", [
        ("t_final", "Duration [s]", 3.0),
        ("dt", "Time step [s]", 0.002),
    ]),
    ("Open-Loop Controller", [
        ("u", "Traction Force, u [N] (Open-Loop Control)", 0.0),
    ]),
    ("Closed-Loop Controller", [
        ("kp", "Kp [N/rad]", 20.0),
        ("ki", "Ki [N/(rad*s)]", 1.0),
        ("kd", "Kd [N*s/rad]", 3.0),
    ]),
]

# Held fixed rather than exposed as editable fields -- gravity is a physical
# constant (not something you'd realistically vary per run), and setpoint/
# u_limit are given sensible defaults here since they were trimmed from the
# PID field group above.
FIXED_GRAVITY = DEFAULT_PARAMS["g"]
FIXED_SETPOINT_DEG = 0.0
FIXED_U_LIMIT = 2.0

class PendulumApp(tk.Tk):
    # Window Initialization
    def __init__(self):
        super().__init__()
        self.title("Stunt Car Inverted Pendulum -- Open-Loop Simulator")
        self.geometry("1350x750")

        self.entries = {}
        self.last_sol = None
        self.mode_var = tk.StringVar(value="uncontrolled")
        self._pid = None
        self._sim_running = False
        self._build_layout()
        self._draw_empty_pole_zero()

    def _build_layout(self):
        # Scrollable left control panel
        left_container = ttk.Frame(self)
        left_container.pack(side=tk.LEFT, fill=tk.Y)

        control_canvas = tk.Canvas(left_container, width=400)
        control_canvas.pack(side=tk.LEFT, fill=tk.Y, expand=False)

        scrollbar = ttk.Scrollbar(
            left_container,
            orient="vertical",
            command=control_canvas.yview
        )
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        control_canvas.configure(yscrollcommand=scrollbar.set)

        # Frame that holds all the controls
        controls = ttk.Frame(control_canvas, padding=10)

        control_window = control_canvas.create_window(
            (0, 0),
            window=controls,
            anchor="nw"
        )

        # Update scroll region whenever contents change
        def update_scroll_region(event):
            control_canvas.configure(
                scrollregion=control_canvas.bbox("all")
            )

        controls.bind("<Configure>", update_scroll_region)

        # Mouse wheel should only scroll the control panel. The previous
        # approach (rebinding on every <Enter>/<Leave>) stacks a NEW
        # bind_all binding each time the cursor crosses between the canvas
        # and any child widget inside it (Entry/Label/Radiobutton), since
        # bind_all does not replace an existing binding -- it adds another
        # one. After enough mouse movement, a single wheel notch would fire
        # the handler many times over, causing exactly the sluggish,
        # over-scrolling behavior being asked about here. Bind once,
        # permanently, and gate on cursor position inside the handler
        # instead of rebinding.
        def on_mousewheel(event):
            widget_under_cursor = self.winfo_containing(event.x_root, event.y_root)
            if widget_under_cursor is None:
                return
            # Only scroll if the cursor is over the control panel itself or
            # one of the widgets packed inside it.
            w = widget_under_cursor
            while w is not None:
                if w == control_canvas:
                    control_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
                    return
                w = w.master

        self.bind_all("<MouseWheel>", on_mousewheel)

        row = 0
        for section_title, fields in FIELD_GROUPS:
            ttk.Label(controls, text=section_title, font=("", 11, "bold")).grid(
                row=row, column=0, columnspan=2, pady=(10 if row else 0, 8), sticky="w"
            )
            row += 1

            for key, label, default in fields:
                ttk.Label(controls, text=label).grid(row=row, column=0, sticky="w", pady=2)
                var = tk.StringVar(value=str(default))
                entry = ttk.Entry(controls, textvariable=var, width=12)
                entry.grid(row=row, column=1, pady=2, padx=(6, 0))
                self.entries[key] = var
                row += 1

        ttk.Label(controls, text="Simulation Mode", font=("", 11, "bold")).grid(
            row=row, column=0, columnspan=2, pady=(10, 2), sticky="w"
        )
        row += 1
        ttk.Radiobutton(
            controls, text="Uncontrolled (u = 0)", variable=self.mode_var, value="uncontrolled"
        ).grid(row=row, column=0, columnspan=2, sticky="w", pady=1)
        row += 1
        ttk.Radiobutton(
            controls, text="Open-Loop (constant u)", variable=self.mode_var, value="open_loop"
        ).grid(row=row, column=0, columnspan=2, sticky="w", pady=1)
        row += 1
        ttk.Radiobutton(
            controls, text="Closed-Loop (PID)", variable=self.mode_var, value="closed_loop"
        ).grid(row=row, column=0, columnspan=2, sticky="w", pady=(1, 4))
        row += 1

        self.simulate_button = ttk.Button(controls, text="Simulate", command=self.run_simulation)
        self.simulate_button.grid(row=row, column=0, columnspan=2, sticky="ew", pady=(4, 4))
        row += 1
        ttk.Button(controls, text="Save Plot...", command=self.save_plot).grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=4
        )
        row += 1

        self.status = ttk.Label(controls, text="", wraplength=200, foreground="#555")
        self.status.grid(row=row, column=0, columnspan=2, pady=(12, 0), sticky="w")
        row += 1

        ttk.Label(controls, text="Recommended PID Controls", font=("", 11, "bold")).grid(
            row=row, column=0, columnspan=2, pady=(14, 2), sticky="w"
        )
        row += 1
        self.recommended_label = ttk.Label(
            controls, text="Run a simulation to see recommended gains.",
            wraplength=200, foreground="#1A6B3C", justify="left",
        )
        self.recommended_label.grid(row=row, column=0, columnspan=2, sticky="w")
        row += 1

        plot_frame = ttk.Frame(self)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(11, 6), dpi=100, constrained_layout=True)
        # 2x3 grid: time-domain plots occupy the left 2x2 block, the
        # pole-zero (s-plane) map spans both rows on the right.
        gs = self.fig.add_gridspec(2, 3, width_ratios=[1, 1, 1])
        self.axes = np.array([
            [self.fig.add_subplot(gs[0, 0]), self.fig.add_subplot(gs[0, 1])],
            [self.fig.add_subplot(gs[1, 0]), self.fig.add_subplot(gs[1, 1])],
        ])
        self.ax_pz = self.fig.add_subplot(gs[:, 2])

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        toolbar = NavigationToolbar2Tk(self.canvas, plot_frame)
        toolbar.update()

    def _simulation_complete(self, sol, mode, params, v):
        """Runs on the MAIN thread (scheduled via self.after) -- this is the
        only place drawing should happen."""
        self._sim_running = False
        self.simulate_button.config(state="normal")

        self.last_sol = sol
        self._draw(sol, mode)
        self._draw_pole_zero(params, v, mode)
        self._update_recommendation(params)

        theta_deg = np.degrees(sol.y[2])
        status_text = (
            f"Final θ: {theta_deg[-1]:.2f}°\n"
            f"Peak |θ|: {np.max(np.abs(theta_deg)):.2f}°"
        )
        if mode == "closed_loop":
            status_text += f"\nPeak |u|: {np.max(np.abs(sol.u)):.3f} N"

        if sol.fell_at is not None:
            status_text = f"FELL at t = {sol.fell_at:.3f}s\n" + status_text
        self.status.config(text=status_text)

    def _simulation_error(self, title, message):
        """Runs on the MAIN thread -- messagebox is not thread-safe either."""
        self._sim_running = False
        self.simulate_button.config(state="normal")
        messagebox.showerror(title, message)

    def run_simulation(self):
        if self._sim_running:
            # Ignore repeat clicks while a simulation is already in flight --
            # letting threads pile up is another way this used to bog down.
            return
        self._sim_running = True
        self.simulate_button.config(state="disabled")
        thread = threading.Thread(target=self._run_simulation_worker, daemon=True)
        thread.start()

    def _run_simulation_worker(self):
        """Runs on a BACKGROUND thread. Only numeric work happens here --
        no Tkinter or matplotlib calls are allowed on this thread, since
        neither is thread-safe. All UI updates are handed back to the main
        thread via self.after(...)."""
        try:
            v = {key: float(self.entries[key].get()) for key in self.entries}
        except ValueError:
            self.after(0, self._simulation_error, "Invalid input", "All fields must be numeric.")
            return

        params = {
            "m_car": v["mass"],
            "l": v["length"],
            "w": v["width"],
            "Cd": v["damping"],
            "g": FIXED_GRAVITY,
        }
        z0 = [v["x0"], v["xdot0"], np.radians(v["theta0"]), np.radians(v["thetadot0"])]

        mode = self.mode_var.get()
        if mode == "closed_loop":
            pid = PID(
                kp=v["kp"], ki=v["ki"], kd=v["kd"],
                setpoint=np.radians(FIXED_SETPOINT_DEG), u_limit=FIXED_U_LIMIT,
            )
            self._pid = pid
            sol = simulate_closed_loop(params, z0, v["t_final"], v["dt"], pid)
        elif mode == "open_loop":
            u_fn = lambda t: v["u"]
            sol = simulate(params, z0, v["t_final"], v["dt"], u_fn)
        else:  # uncontrolled
            u_fn = lambda t: 0.0
            sol = simulate(params, z0, v["t_final"], v["dt"], u_fn)

        if not sol.success:
            self.after(0, self._simulation_error, "Integration failed", sol.message)
            return

        # Hand the finished result back to the main thread for drawing --
        # this is the ONLY place drawing gets triggered (previously it also
        # happened here on the worker thread, which is what caused the lag).
        self.after(0, self._simulation_complete, sol, mode, params, v)

    def _draw(self, sol, mode):
        t = sol.t
        x, x_dot, theta, theta_dot = sol.y
        theta_deg = np.degrees(theta)
        theta_dot_deg = np.degrees(theta_dot)

        for ax in self.axes.flat:
            ax.clear()

        titles = {
            "uncontrolled": "Uncontrolled Response (u = 0)",
            "open_loop": "Open-Loop Response (constant u)",
            "closed_loop": "Closed-Loop (PID) Response",
        }
        self.fig.suptitle(titles[mode])

        self.axes[0, 0].plot(t, x * 1000, color="#1A2033")
        self.axes[0, 0].set_title("Position, x(t)")
        self.axes[0, 0].set_ylabel("x (mm)")
        self.axes[0, 0].grid(alpha=0.3)

        self.axes[0, 1].plot(t, x_dot * 1000, color="#5A5F6E")
        self.axes[0, 1].set_title("Velocity, ẋ(t)")
        self.axes[0, 1].set_ylabel("ẋ (mm/s)")
        self.axes[0, 1].grid(alpha=0.3)

        self.axes[1, 0].plot(t, theta_deg, color="#C0402A")
        self.axes[1, 0].axhline(0, color="black", linewidth=0.6)
        self.axes[1, 0].set_title("Angle, θ(t)")
        self.axes[1, 0].set_ylabel("θ (deg)")
        self.axes[1, 0].set_xlabel("time (s)")
        self.axes[1, 0].grid(alpha=0.3)

        self.axes[1, 1].plot(t, theta_dot_deg, color="#D85A30")
        self.axes[1, 1].set_title("Angular velocity, θ̇(t)")
        self.axes[1, 1].set_ylabel("θ̇ (deg/s)")
        self.axes[1, 1].set_xlabel("time (s)")
        self.axes[1, 1].grid(alpha=0.3)

        if sol.fell_at is not None:
            for ax in self.axes.flat:
                ax.axvline(sol.fell_at, color="black", linewidth=1, linestyle="--", alpha=0.6)
            self.axes[0, 0].annotate(
                "fell", xy=(sol.fell_at, self.axes[0, 0].get_ylim()[1]),
                xytext=(3, -22), textcoords="offset points", fontsize=8, color="#555",
            )

        self.canvas.draw()

    def _draw_empty_pole_zero(self):
        ax = self.ax_pz
        ax.clear()

        ax.axhline(0, color="black", linewidth=0.6)
        ax.axvline(0, color="black", linewidth=0.6)

        ax.axvspan(-5, 0, color="#1A6B3C", alpha=0.06)

        ax.set_title("s-plane -- press Simulate", color="#888888", fontsize=10)
        ax.set_xlabel("Re(s)")
        ax.set_ylabel("Im(s)")
        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
        ax.grid(alpha=0.3)

        self.canvas.draw()

    def _draw_pole_zero(self, params, v, mode):
        """
        Plots the s-plane pole-zero map for the linearized plant:
            G(s) = -B / (s^2 - A)              open-loop poles at s = +-sqrt(A)
        and, using the Kp/Ki/Kd currently entered in the sidebar, the
        closed-loop characteristic equation:
            s^3 + B*kd*s^2 + (B*kp - A)*s + B*ki = 0
        This runs only on the main thread, called from _simulation_complete.
        """
        ax = self.ax_pz
        ax.clear()

        A, B = linearize(params)

        ax.axhline(0, color="black", linewidth=0.6)
        ax.axvline(0, color="black", linewidth=0.6)

        if mode == "uncontrolled":
            # No control input -- nothing to analyze, so no poles are shown.
            ax.axvspan(-5, 0, color="#1A6B3C", alpha=0.06)
            ax.set_title("s-plane -- N/A (uncontrolled, u = 0)", color="#888888", fontsize=10)
            ax.set_xlim(-5, 5)
            ax.set_ylim(-5, 5)
            ax.set_xlabel("Re(s)")
            ax.set_ylabel("Im(s)")
            ax.grid(alpha=0.3)
            self.canvas.draw()
            return

        ol_poles = np.array([np.sqrt(A), -np.sqrt(A)], dtype=complex)

        kd_coeff = B * v["kd"]
        kp_coeff = B * v["kp"] - A
        ki_coeff = B * v["ki"]
        cl_poles = np.roots([1.0, kd_coeff, kp_coeff, ki_coeff])

        if mode == "closed_loop":
            poles = cl_poles
            color = "#1A6B3C"
            label = "Closed-loop poles"
        else:
            poles = ol_poles
            color = "#C0402A"
            label = "Open-loop poles"

        ax.scatter(
            poles.real,
            poles.imag,
            marker="x",
            s=80,
            color=color,
            linewidths=2,
            label=label,
            zorder=3,
        )

        xlim = max(2.0, np.max(np.abs(poles.real)) * 1.3)
        ylim = max(np.max(np.abs(poles.imag)) * 1.3 + 1, xlim * 0.5)
        ax.axvspan(-xlim, 0, color="#1A6B3C", alpha=0.06)

        if mode == "closed_loop":
            stable = np.all(cl_poles.real < 0)
            ax.set_title(
                f"s-plane -- Closed-loop {'STABLE' if stable else 'UNSTABLE'}",
                color="#1A6B3C" if stable else "#C0402A",
                fontsize=10,
            )
        else:
            ax.set_title("s-plane -- Open-loop", fontsize=10)

        ax.set_xlim(-xlim, xlim)
        ax.set_ylim(-ylim, ylim)
        ax.set_xlabel("Re(s)")
        ax.set_ylabel("Im(s)")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.18),
                  fontsize=7, ncol=1, frameon=False)

        # _draw() above already called canvas.draw() once, but that flush
        # happened BEFORE this method updated ax_pz -- without a second
        # flush here, the pole-zero change sits applied-but-unrendered
        # until the next Simulate click catches it up. Flush again now so
        # it shows up on the same click.
        self.canvas.draw()

    def _update_recommendation(self, params):
        """
        Recommends PID gains via pole placement (see pid_tuner.gains_from_p),
        using the physical parameters currently entered in the sidebar.
        Pole location p is chosen as 1.5x the plant's natural instability
        rate sqrt(A) -- fast enough to comfortably out-run the open-loop
        instability without being so aggressive it demands excessive
        control effort.
        """
        A, _ = linearize(params)
        p = 1.5 * np.sqrt(A)
        kp, ki, kd = gains_from_p(p, params)
        self.recommended_label.config(
            text=(
                f"Pole placement (p = {p:.2f} rad/s):\n"
                f"Kp = {kp:.2f} N/rad\n"
                f"Ki = {ki:.2f} N/(rad*s)\n"
                f"Kd = {kd:.3f} N*s/rad\n"
                f"(Try these in the Closed-Loop fields above.)"
            )
        )

    def save_plot(self):
        if self.last_sol is None:
            messagebox.showwarning("Nothing to save", "Run a simulation first.")
            return

        path = filedialog.asksaveasfilename(
            defaultextension=".png",
            filetypes=[("PNG image", "*.png"), ("PDF", "*.pdf"), ("SVG", "*.svg"), ("All files", "*.*")],
            initialfile="pendulum_response.png",
            title="Save plot as...",
        )
        if not path:
            return

        self.fig.savefig(path, dpi=150)
        self.status.config(text=f"Saved to:\n{path}")


if __name__ == "__main__":
    app = PendulumApp()
    app.mainloop()
