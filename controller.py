import numpy as np
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

    def feedForward(self, theta):
        return -self.M_body * self.l * self.g * np.sin(theta)
    
    def balanceControl(variable, theta, thetadot):
        # kp = 5.0
        # kd = 1.765
        kp = 150.0
        kd = 0.55
        # kp = 109.05
        # kd = 24.3
        # print(theta, ((-1*theta*kp)+(-1*thetadot*kd))*1/2)
        return -(((-1*theta*kp)+(-1*thetadot*kd)))*1/2

    def linearVControl(self, v_target, left_wheel, right_wheel):
        kp = 488.58
        # kp = 500
        # ki = 1.80
        # kd = 3.0
        # kp = 120
        ki = 10.0
        kd = 100.0
        v_current = (left_wheel+right_wheel)*self.r/2

        error = v_target - v_current
        print(error)
        proportional = kp * error
        self.v_integral += error * self.dt
        integral = ki * self.v_integral
        derivative = kd * (error - self.v_previous_error) / self.dt
        output = proportional + integral + derivative
        self.v_previous_error = error
        return output*self.r/2
    
    def angularVControl(self, w_target, left_wheel, right_wheel):
        # kp = 40.05
        # ki = 0.003
        # kd = 0.000125
        kp = 0.0
        ki = 0.0
        kd = 0.0
        w_current = (right_wheel-left_wheel)*self.r/self.d
        # print(w_current)

        error = w_target - w_current
        # print(error)
        proportional = kp * error
        self.w_integral += error * self.dt
        integral = ki * self.w_integral
        derivative = kd * (error - self.w_previous_error) / self.dt
        output = proportional + integral + derivative
        self.w_previous_error = error

        return output*self.r/self.d
    
    def compare(self, theta, Tl, Tr):
        M = np.array([[self.M_body + 2*self.M_wheel + 2*self.J/(self.r*self.r), self.M_body*self.l*np.cos(theta[0]), 0],
                    [self.M_body*self.l*np.cos(theta[0]), self.Iy+self.M_body*self.l*self.l, 0],
                    [0, 0, self.Iz+(self.M_wheel*self.d*self.d/2)+(self.J*self.d*self.d/(2*self.r*self.r))]])
        
        b = np.array([[-self.M_body*self.l*theta[1]*theta[1]*np.sin(theta[0])], [0], [0]])

        g = np.array([[0], [-self.M_body*self.g*self.l*np.sin(theta[0])], [0]])

        J = np.array([[1/self.r, 1/self.r],
                     [-1, -1],
                     [-self.d/(2*self.r), self.d/(2*self.r)]])
        
        T = np.array([[Tl], [Tr]])

        qdotdot = np.linalg.inv(M) @ ((J @ T) - b - g)

        return qdotdot
    

