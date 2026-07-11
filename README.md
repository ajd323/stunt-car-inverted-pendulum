# Stunt Car Inverted Pendulum Explicit ODE Formulation

Produced by Andrew J. D'Onofrio (*B.S. MechE '26*)

The following is an control system representation of a stunt car operating as an inverted pendulum. This is inspired by the Cornell University's ECE 4160 / MAE 4190 Fast Robots final project, which requires operating a custom stunt car as a PID-controlled inverted pendulum using ToF and 9DOF IMU sensors. This is a modular program for designs of various geometries and mass, which all operate on a similar dynamics process.

The following is a basic step-by-step rationalization for the dynamics of the instable system, including the implicit and explicit ordinary differential equations (ODEs):

## 1. General Background

### Example Stunt Car Parameters (from Fast Rboots 2026)
&nbsp; Mass (m<sub>car</sub>) = 389.55 g
&nbsp; Length (l<sub>car</sub>) = 175 mm
&nbsp; Width (w<sub>car</sub>) = 140 mm
&nbsp; Height (h<sub>car</sub>) = 75 mm
&nbsp; Radius (r<sub>wheel</sub>) = 75 mm<br>

### Global Definitions

**Coordinates**<br>
&nbsp; *Global Coordinate System*:</strong> <i>X</i><sub>global</sub> = [X, Y, Z] with &theta; about Z <br>

**Dynamic Definitions**<br>
&nbsp; *x (Displacement)* represents the linear translation of the car at the pivot<br>
&nbsp; *&theta; (Theta)* represents the rotation of the car relative to the neutral axis (&theta; = 0)<br>
&nbsp; *State Vector*:</strong> z = [x, ẋ, θ, θ̇]<sup>T</sup><br>
&nbsp; *Input Parameter*:</strong> u = F<sub>traction</sub><br>

**Relevant Force**<br>
&nbsp; *Drag Force*: F<sub>Drag</sub> = v &times; b <br>
&nbsp; *Gravitational Force*: F<sub>Gravity</sub> = m &times; g <br>
&nbsp; *Tractional Force*: F<sub>Traction</sub> = &tau; / R <br>
&nbsp; *Normal Force*: F<sub>Normal</sub> = - m &times; g <br>

**Visual Diagram**<br>
&nbsp; ![System Dynamics Model for Stunt Car](images/FBD_Diagram.png)

## 2. Nonlinearized ODE System

 