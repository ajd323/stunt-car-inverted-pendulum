# Stunt Car Inverted Pendulum Explicit ODE Formulation

Produced by Andrew J. D'Onofrio (*B.S. MechE '26*)

The following is an control system representation of a stunt car operating as an inverted pendulum. This is inspired by the Cornell University's ECE 4160 / MAE 4190 Fast Robots final project, which requires operating a custom stunt car as a PID-controlled inverted pendulum using ToF and 9DOF IMU sensors. This is a modular program for designs of various geometries and mass, which all operate on a similar dynamics process.

The following is a basic step-by-step rationalization for the dynamics of the instable system, including the implicit and explicit ordinary differential equations (ODEs):

## 1. General Background

### Example Stunt Car Parameters (from Fast Rboots 2026)
&nbsp; Mass (m<sub>car</sub>) = 389.55 g<br>
&nbsp; Length (l<sub>car</sub>) = 175 mm<br>
&nbsp; Width (w<sub>car</sub>) = 140 mm<br>
&nbsp; Height (h<sub>car</sub>) = 75 mm<br>
&nbsp; Radius (r<sub>wheel</sub>) = 75 mm<br>

### Global Definitions

**Coordinates**<br>
&nbsp; *Global Coordinate System*:</strong> <i>X</i><sub>global</sub> = [X, Y, Z] with &theta; about Z <br>

**General Definitions**<br>
&nbsp; *x (Displacement)* represents the linear translation of the car at the pivot<br>
&nbsp; *ẋ (Velocity)* represents the linear velocity of the car at the pivot<br>
&nbsp; *θ (Altitude)* represents the rotation of the car relative to the neutral axis (&theta; = 0)<br>
&nbsp; *θ̇ (Angular Velocity)* represents the angular velocity of the car relative to the neutral axis (&theta; = 0)<br>

**Initial Conditions (ICs)**<br>
&nbsp; *Vertical Alignment*: x(0) = 0 mm<br>
&nbsp; *No Vertical Velocity*: ẋ(0) = 0 mm/s<br>
&nbsp; *Pre-Set Angular Discplacement*: θ(0) = 0.5°<br>
&nbsp; *No Angular Velocity*: ẋ(0) = 0°/s<br>

**Relevant Force**<br>
&nbsp; *Drag Force*: F<sub>Drag</sub> = v &times; b <br>
&nbsp; *Gravitational Force*: F<sub>Gravity</sub> = m &times; g <br>
&nbsp; *Tractional Force*: F<sub>Traction</sub> = &tau; / R <br>
&nbsp; *Normal Force*: F<sub>Normal</sub> = - m &times; g <br>

**Visual Diagram**<br>
&nbsp; ![System Dynamics Model for Stunt Car](images/FBD_Diagram.png)

## 2. Nonlinearized ODE System

**General Nonlinear ODE Components**<br>
&nbsp; *State Vector*:</strong> z = [z<sub>1</sub>, z<sub>2</sub>, z<sub>3</sub>, z<sub>4</sub>]<sup>T</sup> = [x, ẋ, θ, θ̇]<br>
&nbsp; *Input Parameter*:</strong> u = F<sub>Traction</sub><br>

**Standard Nonlinear ODE (</strong> z = [z<sub>1</sub>, z<sub>2</sub>, z<sub>3</sub>, z<sub>4</sub>]<sup>T</sup>)**<br>
&nbsp; *Linear Displacement*: z<sub>1</sub> = x<br>
&nbsp; *Linear Velocity*: z<sub>2</sub> = ẋ<br>
&nbsp; *Angular Displacement*: z<sub>3</sub> = θ<br>
&nbsp; *Angular Velocity*: z<sub>4</sub> = θ̇<br>

**Derivative Nonlinear ODE (</strong> ż = [ż<sub>1</sub>, ż<sub>2</sub>, ż<sub>3</sub>, ż<sub>4</sub>]<sup>T</sup>)**<br>
&nbsp; *Linear Velocity*: ż<sub>1</sub> = z<sub>2</sub> = ẋ<br>
&nbsp; *Linear Acceleration*: ż<sub>2</sub> = [ ((1/3)l² + (1/12)w²)·u + b·z<sub>2</sub>·((l²/4)cos²z<sub>3</sub> − ((1/3)l² + (1/12)w²)) + ((1/3)l² + (1/12)w²)·m<sub>car</sub>·(l/2)·sin(z<sub>3</sub>)·z<sub>4</sub>² − (l²/4)·m<sub>car</sub>·g·sin(z<sub>3</sub>)·cos(z<sub>3</sub>) ] / (m<sub>car</sub>·((1/12)(l² + w²) + (l²/4)sin²z<sub>3</sub>))<br>
&nbsp; *Angular Velocity*: ż<sub>3</sub> = z<sub>4</sub> = θ̇<br>
&nbsp; *Angular Acceleration*: ż<sub>4</sub> = (l/2) / ((1/12)(l² + w²) + (l²/4)sin²z<sub>3</sub>) · [ g·sin(z<sub>3</sub>) − u·cos(z<sub>3</sub>)/m<sub>car</sub> − (l/2)·z<sub>4</sub>²·sin(z<sub>3</sub>)·cos(z<sub>3</sub>) ]<br>