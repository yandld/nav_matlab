from math import sin,cos,tan,atan2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


sample_time = 0.01
time_end = 20


class Bicycle():
    def __init__(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0
        self.L = 2
        self.lr = 1.2
        self.w_max = 1.22
        self.sample_time = 0.01
        
    def reset(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0
        
class Bicycle(Bicycle):
    def step(self, v, w):
        xc_dot = v * cos(self.theta + self.beta)
        yc_dot = v * sin(self.theta + self.beta)
        theta_dot = v * cos(self.beta) * tan(self.delta) / self.L
        delta_dot = max(-self.w_max, min(self.w_max, w))
        self.xc += xc_dot * self.sample_time
        self.yc += yc_dot * self.sample_time
        self.theta += theta_dot * self.sample_time
        self.delta += delta_dot * self.sample_time
        self.beta = atan2(self.lr * tan(self.delta), self.L)

model = Bicycle()


model.delta = np.arctan(2/10)
t_data = np.arange(0, time_end, sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)

for i in range(t_data.size):
    x_data[i] = model.xc
    y_data[i] = model.yc
    model.step(np.pi, 0)
    
plt.axis('equal')
plt.plot(x_data, y_data, label='linear model')
plt.legend()
plt.show()

# homwowrk trigrarity
path_radius = 8
sample_time = 0.01
time_end = 30
model.reset()

t_data = np.arange(0, time_end, sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)
v_data = np.zeros_like(t_data)
w_data = np.zeros_like(t_data)

# ==================================
#  Learner solution begins here
# ==================================
n = t_data.shape[0]
max_delta = 0.993*atan2(model.L, path_radius)
v_data[:] = 4 * np.pi * path_radius / time_end
#model.delta = max_delta
#w_data[n//8-50:n//8+50] = -2 * max_delta
#w_data[5*n//8-50:5*n//8+50] = 2 * max_delta
for i in range(n):
    x_data[i] = model.xc
    y_data[i] = model.yc
    if i < n/8 and model.delta < max_delta:
        w_data[i] = model.w_max
    elif n/8 < i < 5*n/8 and model.delta > -max_delta:
        w_data[i] = -model.w_max
    elif i > 5*n/8 and model.delta < max_delta:
        w_data[i] = model.w_max

    model.step(v_data[i], w_data[i])
    
# ==================================
#  Learner solution ends here
# ==================================
plt.axis('equal')
plt.plot(x_data, y_data)
plt.show()



data = np.vstack([t_data, v_data, w_data]).T
np.savetxt('figure8.txt', data, delimiter=', ')

