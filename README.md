# Stunt Car Inverted Pendulum Explicit ODE Formulation
Produced by Andrew J. D'Onofrio (*B.S. MechE '26*)

The following is a control system representation of a stunt car operating as an inverted pendulum. This is inspired by Cornell University's ECE 4160 / MAE 4190 Fast Robots final project, which requires operating a custom stunt car as a PID-controlled inverted pendulum using ToF and 9DOF IMU sensors. This is a modular program for designs of various geometries and masses, which all operate under the same underlying dynamics.

The following is a basic step-by-step rationalization for the dynamics of the unstable system, including the implicit and explicit ordinary differential equations (ODEs):

# Background for "Pendulum_Model.py"

## 1. General Background

### Example Stunt Car Parameters (from Fast Robots 2026)
&nbsp; Mass (m<sub>car</sub>) = 389.55 g<br>
&nbsp; Length (l<sub>car</sub>) = 175 mm<br>
&nbsp; Width (w<sub>car</sub>) = 140 mm<br>
&nbsp; Height (h<sub>car</sub>) = 75 mm<br>
&nbsp; Radius (r<sub>wheel</sub>) = 75 mm<br>

### Global Definitions
**Coordinates**<br>
&nbsp; *Global Coordinate System*: **<i>X</i><sub>global</sub>** = [X, Y, Z] with &theta; about Z <br>

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
&nbsp; *Drag Force*: F<sub>Drag</sub> = (1/2)·&rho;·C<sub>D</sub>·A·ẋ<br>
&nbsp; *Gravitational Force*: F<sub>Gravity</sub> = m·g <br>
&nbsp; *Traction Force*: F<sub>Traction</sub> = &tau;/R <br>
&nbsp; *Normal Force*: F<sub>Normal</sub> = -m·g <br>

**Visual Diagram**<br>
&nbsp; ![System Dynamics Model for Stunt Car](images/FBD_Diagram.png)

### 2nd Order ODE Dynamic Representation of Center of Mass
&nbsp; F<sub>External</sub> = m<sub>car</sub>·a<br>
&nbsp; **F<sub>x</sub>**: F<sub>Traction</sub> - F<sub>Drag</sub> = m<sub>car</sub>·a<sub>x</sub><br>
&nbsp; **F<sub>y</sub>**: F<sub>Gravity</sub> = m<sub>car</sub>·a<sub>y</sub><br>

&nbsp; *Solve for acceleration terms (a<sub>x</sub> and a<sub>y</sub>) in terms of displacement and rotation about the center of mass*:<br>
&nbsp;&nbsp; x<sub>COM</sub> = x<sub>Pivot</sub> + l/2·sin(θ)<br>
&nbsp;&nbsp; y<sub>COM</sub> = l/2·cos(θ)<br>
&nbsp;&nbsp; ẋ<sub>COM</sub> = ẋ<sub>Pivot</sub> + l/2·cos(θ)·θ̇<br>
&nbsp;&nbsp; ẏ<sub>COM</sub> = -l/2·sin(θ)·θ̇<br>
&nbsp;&nbsp; ẍ<sub>COM</sub> = ẍ<sub>Pivot</sub> + l/2·(cos(θ)·θ̈ - sin(θ)·θ̇<sup>2</sup>)<br>
&nbsp;&nbsp; ÿ<sub>COM</sub> = -l/2·(sin(θ)·θ̈ + cos(θ)·θ̇<sup>2</sup>)<br>

&nbsp; **F<sub>x</sub>**: F<sub>Traction</sub> - F<sub>Drag</sub> = m<sub>car</sub>·(ẍ<sub>Pivot</sub> + l/2·(cos(θ)·θ̈ - sin(θ)·θ̇<sup>2</sup>))<br>
&nbsp; **F<sub>y</sub>**: F<sub>Gravity</sub> = -m<sub>car</sub>·l/2·(sin(θ)·θ̈ + cos(θ)·θ̇<sup>2</sup>)<br>
&nbsp; **F<sub>x</sub>**: &tau;/R -  (1/2)·&rho;·C<sub>D</sub>·(l·w)·ẋ = m<sub>car</sub>·(ẍ<sub>Pivot</sub> + l/2·(cos(θ)·θ̈ - sin(θ)·θ̇<sup>2</sup>))<br>
&nbsp; **F<sub>y</sub>**: m·g = -m<sub>car</sub>·l/2·(sin(θ)·θ̈ + cos(θ)·θ̇<sup>2</sup>)<br>

## 2. Nonlinear Form of the ODE System

&nbsp; *State Vector*: **z** = [z<sub>1</sub>, z<sub>2</sub>, z<sub>3</sub>, z<sub>4</sub>]<sup>T</sup> = [x, ẋ, θ, θ̇]<br>
&nbsp; *Input Parameter*: u = &tau;<br>

### Standard Nonlinear ODE (**z** = [z<sub>1</sub>, z<sub>2</sub>, z<sub>3</sub>, z<sub>4</sub>]<sup>T</sup>)<br>
&nbsp; *Linear Displacement*: z<sub>1</sub> = x<br>
&nbsp; *Linear Velocity*: z<sub>2</sub> = ẋ<br>
&nbsp; *Angular Displacement*: z<sub>3</sub> = θ<br>
&nbsp; *Angular Velocity*: z<sub>4</sub> = θ̇<br>

### Derivative Nonlinear ODE (**ż** = [ż<sub>1</sub>, ż<sub>2</sub>, ż<sub>3</sub>, ż<sub>4</sub>]<sup>T</sup>)**<br>
&nbsp; *Linear Velocity*: ż<sub>1</sub> = z<sub>2</sub> = ẋ<br>
&nbsp; *Linear Acceleration*: ż<sub>2</sub> = [ ((1/3)l² + (1/12)w²)·u + C<sub>d</sub>·z<sub>2</sub>·|z<sub>2</sub>|·((l²/4)cos²z<sub>3</sub> − ((1/3)l² + (1/12)w²)) + ((1/3)l² + (1/12)w²)·m<sub>car</sub>·(l/2)·sin(z<sub>3</sub>)·z<sub>4</sub>² − (l²/4)·m<sub>car</sub>·g·sin(z<sub>3</sub>)·cos(z<sub>3</sub>) ] / (m<sub>car</sub>·((1/12)(l² + w²) + (l²/4)sin²z<sub>3</sub>))<br>
&nbsp; *Angular Velocity*: ż<sub>3</sub> = z<sub>4</sub> = θ̇<br>
&nbsp; *Angular Acceleration*: ż<sub>4</sub> = (l/2) / ((1/12)(l² + w²) + (l²/4)sin²z<sub>3</sub>) · [ g·sin(z<sub>3</sub>) − u·cos(z<sub>3</sub>)/m<sub>car</sub> − (l/2)·z<sub>4</sub>²·sin(z<sub>3</sub>)·cos(z<sub>3</sub>) ]<br>

## 3. Linearized State-Space Model

&nbsp; *Equilibrium Point*: z<sub>eq</sub> = [0, 0, 0, 0]<sup>T</sup>, u<sub>eq</sub> = 0 &nbsp;(upright, at rest, no input)<br>

**Linear State-Space Form**<br>
&nbsp; **ż** = A**z** + B**u**<br>

<pre>
z =
[ x ]
[ ẋ ]
[ θ ]
[ θ̇ ]

u = [ &tau; ]

A =
[ 0     1               0              0 ]
[ 0     0          -3gl²/(l²+w²)       0 ]
[ 0     0               0              1 ]
[ 0     0          6gl/(l²+w²)         0 ]

B =
[            0               ]
[ (4l² + w²)/(m_car(l² + w²)) ]
[            0               ]
[  -6l/(m_car(l² + w²))       ]
</pre>

# Background for "Pendulum_Tuner.py"

## 1. LaPlace Transform (from Linearized Model)

**With small angle approximation:** θ̈ = A0 - Bu<br>
&nbsp; θ̈ - A0 = - Bu<br>
&nbsp; L{θ̈} - L {A0} = - {Bu}<br>
&nbsp; s²{θ̈(s)} - A{0(s)} = - B{U(s)}<br>
&nbsp; G(s) = 0(s) / U(s) = - B{U(s)} / s²{θ̈(s)} - A{0(s)}<br>
&nbsp; p<sub>1,2</sub> = ±rad(A)<br>

## 2. Poles of Response Dynamics

**Closed-Loop Characteristic Equation:** K<sub>p</sub> + K<sub>i</sub> / s + K<sub>d</sub> / s<br>
&nbsp; U(s) = C(s)·0(s) = <br>
&nbsp; s³ + B·kd·s² + (B·kp − A)·s + B·ki = 0<br>

## 3. Critical Dampening Ratio
