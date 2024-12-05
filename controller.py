import numpy as np
import math
class Controller:
    def __init__(self, M_body, M_wheel, l, d, Ix, Iy, Iz, J, r, dt, g = -9.81):

        self.dt = dt
        self.M_body = M_body
        self.M_wheel = M_wheel
        self.l = l
        self.d = d
        self.Ix = Ix
        self.Iy = Iy
        self.Iz = Iz
        self.J = J
        self.r = r
        self.g = g

        # Internal variables for PID
        self.v_previous_error = 0
        self.v_integral = 0
        self.w_previous_error = 0
        self.w_integral = 0

    def feedForward(self, theta, target):
        return -self.M_body * self.l * self.g * np.sin(theta-target)
    
    def balanceControl(self, theta, thetadot, target):
        if abs(target) > 0.01:
            kp = 0.1
            kd = 0.015
        else:
            kp = 0.4
            kd = 0.04
        theta_target = target
        thetadot_target = (0-theta_target)/self.dt
        if abs(theta_target-theta) < 0.0001:
            return 0
        else:
            return -(((theta_target-theta)*kp)+((thetadot_target-thetadot)*kd))*1/2

    def linearVControl(self, v_target, left_wheel, right_wheel):
        kp = 0.05
        ki = 0.02
        kd = 0.01
        v_current = (left_wheel+right_wheel)*self.r/2

        error = v_target - v_current
        proportional = kp * error
        self.v_integral += error * self.dt
        integral = ki * self.v_integral
        derivative = kd * (error - self.v_previous_error) / self.dt
        output = proportional + integral + derivative
        self.v_previous_error = error
        return output
    
    def angularVControl(self, w_target, left_wheel, right_wheel):
        kp = 2.0
        ki = 0.4
        kd = 0.08
        w_current = (right_wheel-left_wheel)*self.r/self.d

        error = w_target - w_current
        proportional = kp * error
        self.w_integral += error * self.dt
        integral = ki * self.w_integral
        derivative = kd * (error - self.w_previous_error) / self.dt
        output = proportional + integral + derivative
        self.w_previous_error = error

        return output*self.r/self.d
    
    def positionControl(self, target_X, target_Y, current_X, current_Y, current_yaw):
        Kp = [0.08, 0.3]
        dx = target_X - current_X
        dy = target_Y - current_Y

        alpha = math.atan2(dy, dx)

        d = math.sqrt((dx**2)+(dy**2))
        e = alpha - current_yaw
        e = math.atan2(math.sin(e), math.cos(e))

        vx = float(Kp[0] * d)
        w = float(Kp[1] * e)

        vx = np.clip(vx, -1, 1)
        w = np.clip(w, -0.5, 0.5)

        # if d < math.sqrt((target_X**2)+(target_Y**2))*0.27:
        #     return 0, 0
        # else:
        return vx, w

    def compare(self, theta, Tl, Tr, velocities, wheel_velocities):
        # Mass matrix (same as before)
        M = np.array([
            [self.M_body + 2*self.M_wheel, self.M_body*self.l*np.cos(theta[0]), 0],
            [self.M_body*self.l*np.cos(theta[0]), self.Iy + self.M_body*self.l**2, 0],
            [0, 0, self.Iz + 2*(self.M_wheel*(self.d/2)**2 + self.J*(self.d/2)**2 / self.r**2)]
        ])

        # Coriolis and centrifugal terms (same as before)
        C = np.array([
            [-self.M_body*self.l*(theta[1]**2)*np.sin(theta[0])],
            [0],
            [0]
        ])

        # Gravitational terms (same as before)
        G = np.array([
            [0],
            [self.M_body*self.g*self.l*np.sin(theta[0])],
            [0]
        ])

        # Damping coefficients (adjust values as needed)
        c_x = 0.04
        c_theta = 1
        c_yaw = 0.04

        # Damping forces/torques
        D = np.array([
            [c_x * velocities[0]],
            [c_theta * velocities[1]],
            [c_yaw * velocities[2]]
        ])

        # Friction torque (adjust coefficient as needed)
        min_speed = 0.0005
        b_static = 0.006
        b_dynamics = 0.0034
        # min_speed = 0
        # b_static = 0
        # b_dynamics = 0
        if abs(wheel_velocities[0]) < min_speed:
            Tl_eff = Tl - b_static * wheel_velocities[0]
        else:
            Tl_eff = Tl - b_dynamics * wheel_velocities[0]
        if abs(wheel_velocities[1]) < min_speed:
            Tr_eff = Tr - b_static * wheel_velocities[1]
        else:
            Tr_eff = Tr - b_dynamics * wheel_velocities[1]

        # Input mapping matrix (same as before)
        B = np.array([
            [1/self.r, 1/self.r],
            [-1, -1],
            [-self.d/(2*self.r), self.d/(2*self.r)]
        ])

        T = np.array([[Tl_eff], [Tr_eff]])

        # Compute accelerations including damping
        qdotdot = np.linalg.inv(M).dot(B.dot(T) - C - G - D)

        return qdotdot

    def dx_dt(x_velocity, acceleration):
        return acceleration
    
    def rk4(x, dx_dt, dt):
        k1 = dx_dt(x)
        k2 = dx_dt(x + 0.5 * dt * k1)
        k3 = dx_dt(x + 0.5 * dt * k2)
        k4 = dx_dt(x + dt * k3)
        return x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)