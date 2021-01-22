from math import cos, sin, pi
import numpy as np
from numpy.linalg import inv

def sph_to_cart(epsilon, alpha, r):
    """
    Transform sensor readings to Cartesian coordinates in the sensor frames. 
    """
    p = np.zeros(3)  # Position vector 

    # Your code here
    p[0] = r * cos(alpha) * cos(epsilon)
    p[1] = r * sin(alpha) * cos(epsilon)
    p[2] = r * sin(epsilon)

    return p

def estimate_params(P):
    """
    Estimate parameters from sensor readings in the Cartesian frame.
    Each row in the P matrix contains a single measurement.
    """
    param_est = np.zeros(3)
    A = np.ones_like(P)
    b = np.zeros((P.shape[0], 1))

      # Your code here
    for i, meas in enumerate(P):
      print(meas)
      A[i, 1], A[i, 2], b[i] = meas

    params = inv(A.T @ A) @ A.T @ b
    param_est[0] = params[0, 0]
    param_est[1] = params[1, 0]
    param_est[2] = params[2, 0]

    return param_est

meas = np.array([[pi/3, 0, 5],
                 [pi/4, pi/4, 7],
                 [pi/6, pi/2, 4],
                 [pi/5, 3*pi/4, 6],
                 [pi/8, pi, 3]])

P = np.empty(shape=[0, 3])
for row in meas:
  P = np.vstack([P, sph_to_cart(row[0], row[1], row[2])])

#P = np.array([sph_to_cart(*row) for row in meas])
print(estimate_params(P))