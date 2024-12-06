# Two-wheels self-balancing robot
- This project simulates dynamics of a two-wheels self-balancing robot, and including PID controller to maintain balance and reach the target velocity. 

# Table of content
- [**Installation**](#installation)
- [**Methodology**](#methodology)
- [**User Guide**](#userguide)
- [**Demos & Result**](#demosnresult)
- [**Conclusion**](#conclusion)
- [**Future plan**](#futureplans)
- [**References**](#references)

# Installation <a name = "installation"> </a>
```
pip install pybullet
```

# Methodology <a name = "methodology"> </a>
## System Modeling
The simulation involves modeling the dynamics of a Two-Wheeled Inverted Pendulum Robot (TWIP) system. The key components include:
<br>Two-Wheeled Inverted Pendulum Robot dynamics

#### Parameters
- $x$ &ensp;&emsp;: Position of robot in robot x-axis
- ${\psi}$ &emsp; : Angle of robot around z-axis
- ${\theta}$ &emsp;&nbsp; : Angle of body around wheel rotation axis
- $M_b$ &nbsp; : Mass of body
- $M_w$ &nbsp;: Mass of each wheel
- $r$ &ensp;&emsp;: Radius of wheel
- $l$ &ensp;&emsp; : Distance between body Center of mass and wheel rotation axis
- $d$ &ensp;&emsp;: Distance between 2 wheel
- $I_y$ &ensp;&ensp;: Moment inertia around body y-axis = $\frac{1}{12}M_b(width_x^{2} + width_z^{2})$
- $I_z$ &ensp;&ensp;: Moment inertia around body z-axis = $\frac{1}{12}M_b(width_y^{2} + width_z^{2})$
- $J$ &emsp; : Moment inertia around wheel rotation axis = $\frac{1}{2}M_wr^{2}$
- ${\tau_l}$ &emsp; : Torque apply to left wheel
- ${\tau_r}$ &emsp;: Torque apply to right wheel

#### Kinetic energy
Body kinetic energy
- $T = \frac{1}{2}M_b(\dot{x}^{2} + 2lcos{\theta}\dot{\theta}\dot{x} + (l\dot{\theta})^2) + \frac{1}{2}(I_y\dot{\theta}^{2} + I_z\dot{\psi}^{2} + \frac{1}{2}M_wd^{2}{\psi}^{2})$

Wheel kinetic energy
- $T =M_w\dot{x}^2 + J(\frac{\dot{x}\pm\frac{d}{2}\dot{\psi}}{r})^2$ 

#### Potential energy
- $V = mglcos{\theta}$
#### Lagrange Method
from $L = T-V$
- $L = \frac{1}{2}(M_b+2M_w+\frac{2J}{r^2})\dot{x}^2+\frac{1}{2}(I_z+\frac{M_wd^2}{2}+\frac{Jd^2}{2r^2})\dot{\psi}^2+\frac{1}{2}(I_y+M_bl^2)\dot{\theta}^2+M_b\dot{x}\dot{\theta}lcos{\theta}-M_bglcos{\theta}$
#### Dynamics of TWIP
from $q = [x, {\theta}, {\psi}]$
and ${\tau} = \frac{d}{dt}(\frac{\partial L}{\partial \dot{q}})-\frac{\partial L}{\partial q}$
$\begin{bmatrix}
   M_b+2M_w+\frac{2J}{r^2} & M_blcos{\theta} & 0 \\
   M_blcos{\theta} & I_y+M_bl^2 & 0 \\
   0 & 0 & I_z+\frac{M_wd^2}{2}+\frac{Jd^2}{2r^2}
\end{bmatrix} 
\begin{bmatrix}
    \ddot{x} \\
    \ddot{\theta} \\
    \ddot{\psi}
\end{bmatrix}+
\begin{bmatrix}
    -M_bl\dot{\theta}^2sin{\theta} \\
    0 \\
    0
\end{bmatrix} +
\begin{bmatrix}
    0 \\
    -M_bglsin{\theta}  \\
    0
\end{bmatrix} = 
\begin{bmatrix}
    {\tau_x} \\
    {\tau_{\theta}}  \\
    {\tau_{\psi}}
\end{bmatrix}$ 

calculate ${\tau_x}, {\tau_{\theta}}, {\tau_{\psi}}$ in term of ${\tau_l}, {\tau_r}$
- $\begin{bmatrix}
   M_b+2M_w+\frac{2J}{r^2} & M_blcos{\theta} & 0 \\
   M_blcos{\theta} & I_y+M_bl^2 & 0 \\
   0 & 0 & I_z+\frac{M_wd^2}{2}+\frac{Jd^2}{2r^2}
\end{bmatrix} 
\begin{bmatrix}
    \ddot{x} \\
    \ddot{\theta} \\
    \ddot{\psi}
\end{bmatrix}+
\begin{bmatrix}
    -M_bl\dot{\theta}^2sin{\theta} \\
    0 \\
    0
\end{bmatrix} +
\begin{bmatrix}
    0 \\
    -M_bglsin{\theta}  \\
    0
\end{bmatrix} = 
\begin{bmatrix}
    \frac{1}{r} & \frac{1}{r} \\
    -1 & -1 \\
    \frac{-d}{2r} & \frac{d}{2r}
\end{bmatrix}
\begin{bmatrix}
    {\tau_l} \\
    {\tau_r}
\end{bmatrix}$ 



# User Guide <a name = "userguide"> </a>

# Demos & Result <a name = "demosnresult"> </a>

# Conclusion <a name = "conclusion"> </a>

# Future plan <a name = "futureplans"> </a>

# References <a name = "references"> </a>
