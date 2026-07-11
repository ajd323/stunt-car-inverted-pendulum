# Stunt Car Inverted Pendulum Explicit ODE Formulation

The following is an control system representation of a stunt car operating as an inverted pendulum. This is inspired by the Cornell University's ECE 4160 / MAE 4190 Fast Robots final project, which requires operating a custom stunt car as a PID-controlled inverted pendulum using ToF and 9DOF IMU sensors. This is a modular program for designs of various geometries and mass, which all operate on a similar dynamics process.

The following is a basic step-by-step rationalization for the dynamics of the instable system, including the implicit and explicit ordinary differential equations (ODEs):

## 1. General Background

### Example Stunt Car Parameters (from Fast Rboots 2026)
Mass (m) = 389.55 g; Length (l) = 175 mm; Width (w) = 140 mm; Height (h) = 75 mm <br>

## Free Body Diagram Components

F<sub>Drag</sub> = v &times; b <br>
F<sub>Gravity</sub> = m &times; g <br>
F<sub>Traction</sub> =
<math>
  <mfrac>
    <mi>&tau;</mi>
    <mi>R</mi>
  </mfrac>
</math><br>


## 2. Nonlinearized ODE System