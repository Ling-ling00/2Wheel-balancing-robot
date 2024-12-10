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
Please Check you have all this required:
1. pybullet
2. numpy
3. time
4. matplotlib

if you dont have you can install below code
```
pip install pybullet
pip install matplotlib
pip install time
pip install numpy
```

Clone this Repositories
```
git clone https://github.com/Ling-ling00/2Wheel-balancing-robot.git
```

# Methodology <a name = "methodology"> </a>
## System Modeling
The simulation involves modeling the dynamics of a Two-Wheeled Inverted Pendulum Robot (TWIP) system. The key components include:
<br>Two-Wheeled Inverted Pendulum Robot dynamics

#### Robot dimensions
![alt text](img/dimension.png)
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


![alt text](img/Coordinate.jpg)
#### Kinetic energy
Body kinetic energy

$$T = \frac{1}{2}M_b(\dot{x}^{2} + 2lcos{\theta}\dot{\theta}\dot{x} + (l\dot{\theta})^2) + \frac{1}{2}(I_y\dot{\theta}^{2} + I_z\dot{\psi}^{2} + \frac{1}{2}M_wd^{2}{\psi}^{2})$$

Wheel kinetic energy

$$T =M_w\dot{x}^2 + J(\frac{\dot{x}\pm\frac{d}{2}\dot{\psi}}{r})^2$$

#### Potential energy
$$V = mglcos{\theta}$$
#### Lagrange Method
from $L = T-V$

$$L = \frac{1}{2}(M_b+2M_w+\frac{2J}{r^2})\dot{x}^2+\frac{1}{2}(I_z+\frac{M_wd^2}{2}+\frac{Jd^2}{2r^2})\dot{\psi}^2+\frac{1}{2}(I_y+M_bl^2)\dot{\theta}^2+M_b\dot{x}\dot{\theta}lcos{\theta}-M_bglcos{\theta}$$
#### Dynamics of TWIP
from $q = [x, {\theta}, {\psi}]$
and ${\tau} = \frac{d}{dt}(\frac{\partial L}{\partial \dot{q}})-\frac{\partial L}{\partial q}$
```math
\begin{bmatrix}
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
\end{bmatrix}
```

calculate ${\tau_x}, {\tau_{\theta}}, {\tau_{\psi}}$ in term of ${\tau_l}, {\tau_r}$
```math
\begin{bmatrix}
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
\end{bmatrix}
```

## System Architecture
In this simulation control a Two-Wheeled Inverted Pendulum Robot (TWIP) with PID control and feed forward.

![alt text](img/system_architecture.png)

- **Target position:** provides the target position in X-Y axis for the robot. 
- **Target linear velocity:** provides the target linear velocity for the robot. 
- **Target angular velocity:** provides the target angular velocity for the robot. 
- **Cartesian to polar coordinates:** Calculate polar coordinates between robot and target from cartesian coordinates. 
- **Position PID control:** Using PID control to generates angular velocity and linear velocity based on distance between robot and target. 
- **Inverted pendulum Feed Forward:** get input that is current body rotation around y-axis and calculate torque to counter gravitational terms. 
- **Inverted pendulum PD control:** Using PD control to generates torque for each wheel based on body rotation around y-axis.
- **Robot rotation PID control:** using diff-drive kinematics to calculate current angular velocity of robot, compare to target velocity and using PID control to generate wheel torque for the left and right wheels. 
- **Robot translation PID control:** using diff-drive kinematics to calculate current linear velocity of robot, compare to target velocity and using PID control to generate robot pitch angle (cascade control). 
- **Plant:** Receive a torque, which is a combine torques from the Inverted pendulum Feed Forward, Inverted pendulum PD control and Robot rotation PID control, of each wheel and return output as rotation of each wheel, tilt angle and position of robot in X-Y plane. 

# User Guide <a name = "userguide"> </a>
### Velocity and Position control Simulation
1. Open File simulation.py
2. Run the file
- You will see interface like this

![interface](img/interface.png)

**1. Input Zone:** Adjust Target of Position and Velocity.
**2. Switch Button:** Use to Switch Between Control Position target or Velocity Target.
>**This 2 mode is seperatly out.**

**3. Simulation Zone:** Show the simulation.

After Simulation Finish You will see Graph
![speed graph](img/balance_graph.png)
![position graph](img/pos_graph.png)
**1. Forward Velocity:** Plotting linear velocity over time.
**2. Pitch Velocity:** Plotting pitch velocity over time.
**3. Yaw Velocity:** Plotting Yaw velocity around z-axis over time.
**4. Left Torque:** Plotting torque left wheel over time.
**5. Right Torque:** Plotting torque Right wheel over time.
**6. X-Position:** Plotting Position robot move in X-axis over time.
**7. Y-Position:** Plotting Position robot move in Y-axis over time.

### Equation compare Simulation
1. Open File equation_compare.py
2. Run the file
- You will see interface like this
![alt text](img/eq_com_interface.png)

**1. Linear Speed:** Adjust Target linear velocity.
**2. Angular Speed:** Adjust Target angular velocity.
**3. Start Button:** Start equation compare simulation and simulation will run 10 second and show the graph.

After Simulation Finish You will see Graph
![alt text](img/compare_no_fric.png)

**1. Real X Velocity:** Plotting linear velocity from simulation over time.
**2. Real Pitch Velocity:** Plotting pitch velocity from simulation over time.
**3. Real Yaw Velocity:** Plotting Yaw velocity around z-axis from simulation over time.
**4. Calculated X Velocity:** Plotting linear velocity from equation over time.
**5. Calculated Pitch Velocity:** Plotting pitch velocity from equation over time.
**6. Calculated Yaw Velocity:** Plotting Yaw velocity around z-axis from equation over time.

# Demos & Result <a name = "demosnresult"> </a>
## 1. Self-Balancing
This Demo will show our robot can self-balance itself.

https://github.com/user-attachments/assets/5121acdb-f3fc-406e-a9b1-435820a80894

plotting graph to visualize.

![alt text](img/balance_graph.png)

Frist we add external force to robot from the video will see it can balance itself and from pitch graph it closed to **0 rad/s** after 30 second that mean robot balancing finish .

## 2. Linear Velocity Control
This Demo will show our robot acheive linear velocity target from input.

https://github.com/user-attachments/assets/8ae4eff4-0120-4ed7-bb46-2bfe011fda2f

plotting graph to visualize.

![alt text](img/Linear_graph.png)

after input target to **1 m/s** this robot use 20 second to acheive target and after change target to **-1 m/s** it use 20 second to aceive target too. 

## 3. Angular Velocity Control
This Demo will show our robot acheive yaw velocity target from input.

https://github.com/user-attachments/assets/988f17a6-996c-4843-91b9-7d9c07aa536c

plotting graph to visualize.

![alt text](img/yaw_graph.png)
Input Angular Velocity to **0.5 rad/s** after robot acheive target input new target to **-0.5 rad/s** from the graph will see they can acheive target too.

## 4. Linear and Angular Velocity Control
This Demo will show our robot acheive linear and angular velocity target from input.

https://github.com/user-attachments/assets/d02d0f99-42c9-48c8-af1c-3f6831599d21

plotting graph to visualize.
![alt text](img/lin_yaw_graph.png)

when input linear velocity to **1 m/s** and input Angular velocity to **0.5 rad/s** and they can acheive target both.

## 5. Position Control
This demo show robot can acheive position target in X and Y axis

https://github.com/user-attachments/assets/16d5feb2-e80b-411a-b317-b721595c6b67

plotting graph to visualize.
![alt text](img/pos_graph.png)

from this demo first we input target position in X-axis to **3 m** and after it closed to target we apply Y position to **-3 m** the result is robot can acheive both target. 

## 6. Compare Simulation with the equation.
To ensure the simulation is equal the equation we use this dynamics equation

```math
\begin{bmatrix}
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
\end{bmatrix}
```

demo simulation

https://github.com/user-attachments/assets/5402b1d8-da1a-4db4-8e29-a80277e5b849

plotting graph to visualize.
![alt text](img/compare_no_fric.png)
the result is calculation and simulation have some trend before 4 second. 

So we add friction term in the equation and compare again.
This is the new equation after add friction term.
```math
\begin{bmatrix}
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
    {\tau_l - f_l} \\
    {\tau_r - f_r}
\end{bmatrix}
```
this is graph after add friction term
![alt text](img/compare_with_fric.png)
from this graph will see it have more likely simulation but not all. That make us think this equation must to have disturbance term that come from physics engine so we can summarized this equation in this form.

```math
\begin{bmatrix}
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
    {\tau_l - f_l} \\
    {\tau_r - f_r}
\end{bmatrix} -
\begin{bmatrix}
    {\tau_{disturbance}}\\
    {\tau_{disturbance_{\theta}}}\\
    {\tau_{disturbance_{\psi}}}
\end{bmatrix}
```

# Conclusion <a name = "conclusion"> </a>

# Future plan <a name = "futureplans"> </a>
- Fine tuning PID for more precision control.
- Finding disturbance term in equation.

# References <a name = "references"> </a>
- [1] Kathryn Remell. “Mathematical Modeling of a Two Wheeled Robotic Base” Uark ScholarWorks, 2021

- [2] Nguyen Cao Cuong., Hoang Dinh Co. “Balancing and Trajectory Tracking Control for Two Wheeled Self-Balancing Robot”, 2024

- [3] Sangtae Kim., SangJoo Kwon. “On the Dynamic Model of a Two-Wheeled Inverted Pendulum Robot”, 2014

