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
from pendulum_model import DEFAULT_PARAMS, simulate

# User Interface Parameters
FIELD_GROUPS = [
    ("Stunt Car Geometric / Physical Parameters", [
        ("mass", "Mass [kg]", DEFAULT_PARAMS["m_car"]),
        ("length", "Length [m]", DEFAULT_PARAMS["l"]),
        ("width", "Width [m]", DEFAULT_PARAMS["w"]),
        ("damping", "Drag Coefficient [N*s^2/m^2]", DEFAULT_PARAMS["Cd"]),
        ("gravity", "Gravity [m/s^2]", DEFAULT_PARAMS["g"]),
    ]),
    ("Initial Conditions", [
        ("x0", "Initial Position [m]", 0.0),
        ("xdot0", "Initial Velocity [m/s]", 0.0),
        ("theta0", "Initial Angle [deg]", 0.5),
        ("thetadot0", "Initial Angular Velocity [deg/s]", 0.0),
    ]),
    ("Simulation Settings", [
        ("u", "Traction Force, u [N] (Open-Loop Control)", 0.0),
        ("t_final", "Duration [s]", 3.0),
        ("dt", "Time step [s]", 0.002),
    ]),
]

class PendulumApp(tk.Tk):
    # Window Initialization
    def __init__(self):
        super().__init__()
        self.title("Stunt Car Inverted Pendulum (Open-Loop Simulator)")
        self.geometry("1100x750")

        self.entries = {}
        self.last_sol = None
        self._build_layout()
        self.run_simulation()

    def _build_layout(self):
        # Input Parameters on Left Side of Window
        controls = ttk.Frame(self, padding=10)
        controls.pack(side=tk.LEFT, fill=tk.Y)
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
        # Relevant Buttons for Simulation and Saving Plots
        ttk.Button(controls, text="Simulate", command=self.run_simulation).grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=(12, 4)
        )
        row += 1
        ttk.Button(controls, text="Save Plot...", command=self.save_plot).grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=4
        )
        row += 1

        # Input Parameters on Left Side of Window
        self.status = ttk.Label(controls, text="", wraplength=200, foreground="#555")
        self.status.grid(row=row, column=0, columnspan=2, pady=(12, 0), sticky="w")
        plot_frame = ttk.Frame(self)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.fig = Figure(figsize=(8, 6), dpi=100, constrained_layout=True)
        self.axes = self.fig.subplots(2, 2)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        toolbar = NavigationToolbar2Tk(self.canvas, plot_frame)

        toolbar.update()

    def run_simulation(self):
        # Read Input Parameters and Validate
        try:
            v = {key: float(self.entries[key].get()) for key in self.entries}
        except ValueError:
            messagebox.showerror("Invalid input", "All fields must be numeric.")
            return

        params = {
            "m_car": v["mass"],
            "l": v["length"],
            "w": v["width"],
            "Cd": v["damping"],
            "g": v["gravity"],
        }
        z0 = [v["x0"], v["xdot0"], np.radians(v["theta0"]), np.radians(v["thetadot0"])]
        u_fn = lambda t: v["u"]

        solution = simulate(params, z0, v["t_final"], v["dt"], u_fn)
        if not solution.success:
            messagebox.showerror("Integration failed", solution.message)
            return

        self.last_sol = solution
        self._draw(solution)

        theta_deg = np.degrees(solution.y[2])
        status_text = f"Final θ: {theta_deg[-1]:.2f}°\nPeak |θ|: {np.max(np.abs(theta_deg)):.2f}°"
        if solution.fell_at is not None:
            status_text = f"FELL at t = {solution.fell_at:.3f}s\n" + status_text
        self.status.config(text=status_text)

    def _draw(self, solution):
        t = solution.t
        x, x_dot, theta, theta_dot = solution.y
        theta_deg = np.degrees(theta)
        theta_dot_deg = np.degrees(theta_dot)

        for ax in self.axes.flat:
            ax.clear()

        # Position Graph
        self.axes[0, 0].plot(t, x * 1000, color="#1A2033")
        self.axes[0, 0].set_title("Position, x(t)")
        self.axes[0, 0].set_ylabel("x (mm)")
        self.axes[0, 0].grid(alpha=0.3)
        # Velocity Graph
        self.axes[0, 1].plot(t, x_dot * 1000, color="#5A5F6E")
        self.axes[0, 1].set_title("Velocity, ẋ(t)")
        self.axes[0, 1].set_ylabel("ẋ (mm/s)")
        self.axes[0, 1].grid(alpha=0.3)
        # Angular Graph
        self.axes[1, 0].plot(t, theta_deg, color="#C0402A")
        self.axes[1, 0].axhline(0, color="black", linewidth=0.6)
        self.axes[1, 0].set_title("Angle, θ(t)")
        self.axes[1, 0].set_ylabel("θ (deg)")
        self.axes[1, 0].set_xlabel("time (s)")
        self.axes[1, 0].grid(alpha=0.3)
        # Angular Velocity Graph
        self.axes[1, 1].plot(t, theta_dot_deg, color="#D85A30")
        self.axes[1, 1].set_title("Angular velocity, θ̇(t)")
        self.axes[1, 1].set_ylabel("θ̇ (deg/s)")
        self.axes[1, 1].set_xlabel("time (s)")
        self.axes[1, 1].grid(alpha=0.3)

        # Fall Detection and Annotation
        if solution.fell_at is not None:
            for ax in self.axes.flat:
                ax.axvline(solution.fell_at, color="black", linewidth=1, linestyle="--", alpha=0.6)
            self.axes[0, 0].annotate(
                "fell", xy=(solution.fell_at, self.axes[0, 0].get_ylim()[1]),
                xytext=(3, -12), textcoords="offset points", fontsize=8, color="#555",
            )
        self.canvas.draw()

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