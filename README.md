# Stunt Car Inverted Pendulum Explicit ODE Formulation
Produced by Andrew J. D'Onofrio (*B.S. MechE '26*)

The following is a control system representation of a stunt car operating as an inverted pendulum. This is inspired by Cornell University's ECE 4160 / MAE 4190 Fast Robots final project, which requires operating a custom stunt car as a PID-controlled inverted pendulum using ToF and 9DOF IMU sensors. This is a modular program for designs of various geometries and masses, which all operate under the same underlying dynamics.

The following is a basic step-by-step rationalization for the dynamics of the unstable system, including the implicit and explicit ordinary differential equations (ODEs):

## 1. General Background

### Example Stunt Car Parameters (from Fast Robots 2026)
&nbsp; Mass (m<sub>car</sub>) = 389.55 g<br>
&nbsp; Length (l<sub>car</sub>) = 175 mm<br>
&nbsp; Width (w<sub>car</sub>) = 140 mm<br>
&nbsp; Height (h<sub>car</sub>) = 75 mm<br>
&nbsp; Radius (r<sub>wheel</sub>) = 75 mm<br>

### Global Definitions
**Coordinates**<br>
&nbsp; *Global Coordinate System*: <i>X</i><sub>global</sub> = [X, Y, Z] with &theta; about Z <br>

**General Definitions**<br>
&nbsp; *x (Displacement)* represents the linear translation of the car at the pivot<br>
&nbsp; *ẋ (Velocity)* represents the linear velocity of the car at the pivot<br>
&nbsp; *θ (Angular Displacement)* represents the rotation of the car relative to the neutral axis (&theta; = 0)<br>
&nbsp; *θ̇ (Angular Velocity)* represents the angular velocity of the car relative to the neutral axis (&theta; = 0)<br>

**Initial Conditions (ICs)**<br>
&nbsp; *Horizontal Alignment*: x(0) = 0 mm<br>
&nbsp; *No Horizontal Velocity*: ẋ(0) = 0 mm/s<br>
&nbsp; *Pre-Set Angular Displacement*: θ(0) = 0.5°<br>
&nbsp; *No Angular Velocity*: θ̇(0) = 0°/s<br>

**Relevant Forces**<br>
&nbsp; *Drag Force*: F<sub>Drag</sub> = ẋ·b <br>
&nbsp; *Gravitational Force*: F<sub>Gravity</sub> = m·g <br>
&nbsp; *Traction Force*: F<sub>Traction</sub> = &tau;/R <br>
&nbsp; *Normal Force*: F<sub>Normal</sub> = -m·g <br>

**Visual Diagram**<br>
&nbsp; ![System Dynamics Model for Stunt Car](images/FBD_Diagram.png)

**2nd Order ODE Dynamic Representation of Center of Mass**
&nbsp; F<sub>External</sub> = m<sub>car</sub>·a
&nbsp; **F<sub>x</sub>**: F<sub>Traction</sub> - F<sub>Drag</sub> = m<sub>car</sub>·a<sub>x</sub>
&nbsp; **F<sub>y</sub>**: F<sub>Gravity</sub> = m<sub>car</sub>·a<sub>y</sub>
&nbsp; *Solve for acceleration terms (a<sub>x</sub> and a<sub>y</sub>) in terms of displacement and rotation about the center of mass*:
&nbsp;&nbsp; x<sub>COM</sub> = x<sub>Pivot</sub> + l/2·sin(θ)
&nbsp;&nbsp; y<sub>COM</sub> = l/2·cos(θ)
&nbsp;&nbsp; ẋ<sub>COM</sub> = ẋ<sub>Pivot</sub> + l/2·cos(θ)·θ̇
&nbsp;&nbsp; ẏ<sub>COM</sub> = -l/2·sin(θ)·θ̇
&nbsp;&nbsp; ẍ<sub>COM</sub> = ẍ<sub>Pivot</sub> + l/2·(cos(θ)·θ̈ - sin(θ)·θ̇<sup>2</sup>)
&nbsp;&nbsp; ÿ<sub>COM</sub> = -l/2·(sin(θ)·θ̈ + cos(θ)·θ̇<sup>2</sup>)
&nbsp; **F<sub>x</sub>**: F<sub>Traction</sub> - F<sub>Drag</sub> = m<sub>car</sub>·(ẍ<sub>Pivot</sub> + l/2·(cos(θ)·θ̈ - sin(θ)·θ̇<sup>2</sup>))
&nbsp; **F<sub>y</sub>**: F<sub>Gravity</sub> = -m<sub>car</sub>·l/2·(sin(θ)·θ̈ + cos(θ)·θ̇<sup>2</sup>)
&nbsp; **F<sub>x</sub>**: &tau;/R - ẋ·b = m<sub>car</sub>·(ẍ<sub>Pivot</sub> + l/2·(cos(θ)·θ̈ - sin(θ)·θ̇<sup>2</sup>))
&nbsp; **F<sub>y</sub>**: m·g = -m<sub>car</sub>·l/2·(sin(θ)·θ̈ + cos(θ)·θ̇<sup>2</sup>)