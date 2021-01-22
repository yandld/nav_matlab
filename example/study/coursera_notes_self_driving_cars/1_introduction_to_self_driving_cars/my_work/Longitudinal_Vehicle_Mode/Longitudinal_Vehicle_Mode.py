import sys
from math import sin, atan2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Vehicle():
    def __init__(self):
        #Throttle to engle torque
        self.a_0 = 400
        self.a_1 = 0.1
        self.a_2 = -0.0002
        
        # Gear ratio, effective radius, mass+ inertia
        self.GR = 0.35
        self.r_e = 0.3
        self.J_e = 10
        self.m = 2000
        self.g = 9.81
        
        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.01
        
        #Tire force
        self.c = 10000
        self.F_max = 10000
        
        # State variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0
        
        self.sample_time = 0.01
        
    def reset(self):
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0
        
class Vehicle(Vehicle):
    def step(self, throttle, alpha):
        # ==================================
        #  Implement vehicle model here
        # ==================================
        x_dot = self.v
        v_dot = self.a
        w_e_dot = self.w_e_dot
        self.x += x_dot * self.sample_time
        self.v += v_dot * self.sample_time
        self.w_e += w_e_dot * self.sample_time
        omega_w = self.GR * self.w_e # Wheel speed from engine speed
        s = omega_w * self.r_e / self.v - 1 # Slip ratio
        F_x = self.c * s if abs(s) < 1 else self.F_max
        F_g = self.m * self.g * sin(alpha)
        R_x = self.c_r1 * self.v
        F_aero = self.c_a * self.v**2
        F_load = F_aero + R_x + F_g
        T_e = throttle * (self.a_0 + self.a_1 * self.w_e + self.a_2 * self.w_e**2)
        self.a = (F_x - F_load) / self.m
        self.w_e_dot = (T_e - self.GR * self.r_e * F_load) / self.J_e
        
sample_time = 0.01
time_end = 100
model = Vehicle()

t_data = np.arange(0, time_end, sample_time)
v_data = np.zeros_like(t_data)

# throttle percentage btween 0 and 1
throttle = 0.2

#incline angle(in radians)
alpha = 0* np.pi / 180

for i in range(t_data.size):
    v_data[i] = model.v
    model.step(throttle, alpha)
    
plt.plot(t_data, v_data)
plt.show()
 

time_end = 20
t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)
v_data = np.zeros_like(t_data)
throttle_data = np.zeros_like(t_data)
alpha_data = np.zeros_like(t_data)

# reset the states
model.reset()

n = t_data.size
throttle_data[:n//4] = [0.2 + 0.3*i/(n//4) for i in range(n//4)]
throttle_data[n//4:3*n//4] = 0.5
throttle_data[3*n//4:] = [0.5 - 0.001*i for i in range(n//4)]


for i in range(n):
    x_data[i] = model.x
    v_data[i] = model.v
    if model.x < 60:
        alpha_data[i] = atan2(3,60)
    elif model.x < 150:
        alpha_data[i] = atan2(9, 90)
    model.step(throttle_data[i], alpha_data[i])

plt.plot(t_data, x_data)
plt.show()

data = np.vstack([t_data, x_data]).T
np.savetxt('xdata.txt', data, delimiter=', ')

    
    
    
    
    

        