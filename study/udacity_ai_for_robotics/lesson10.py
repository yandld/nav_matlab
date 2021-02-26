# Make a robot called myrobot that starts at
# coordinates 30, 50 heading north (pi/2).
# Have your robot turn clockwise by pi/2, move
# 15 m, and sense. Then have it turn clockwise
# by pi/2 again, move 10 m, and sense again.
#
# Your program should print out the result of
# your two sense measurements.
#
# Don't modify the code below. Please enter
# your code at the bottom.

from math import *
import random
import numpy as np

# --------
# 
# the "world" has 4 landmarks.
# the robot's initial coordinates are somewhere in the square
# represented by the landmarks.
#
# NOTE: Landmark coordinates are given in (y, x) form and NOT
# in the traditional (x, y) format!

landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]] # position of 4 landmarks
world_size = 100.0 # world is NOT cyclic. Robot is allowed to travel "out of bounds"
max_steering_angle = pi/4 # You don't need to use this value, but it is good to keep in mind the limitations of a real car.
bearing_noise  = 0.1
steering_noise = 0.1
distance_noise = 5.0

# ------------------------------------------------
# 
# this is the robot class
#

class robot:

    # --------

    # init: 
    #	creates robot and initializes location/orientation 
    #

    def __init__(self, length = 10.0):
        self.x = random.random() * world_size # initial x position
        self.y = random.random() * world_size # initial y position
        self.orientation = random.random() * 2.0 * pi # initial orientation
        self.length = length # length of robot
        self.bearing_noise  = 0.0 # initialize bearing noise to zero
        self.steering_noise = 0.0 # initialize steering noise to zero
        self.distance_noise = 0.0 # initialize distance noise to zero
    
    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))
    # --------
    # set: 
    #	sets a robot coordinate
    #

    def set(self, new_x, new_y, new_orientation):

        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise(ValueError, 'Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def sense(self, add_noise = 1): #do not change the name of this function
        Z = []
        for i in range(len(landmarks)):
            bearing = atan2(landmarks[i][0] - self.y,
                            landmarks[i][1] - self.x) - self.orientation

            if add_noise:
                bearing += random.gauss(0.0, self.bearing_noise)
            bearing %= 2.0*pi
            Z.append(bearing)            

        # ENTER CODE HERE
        # HINT: You will probably need to use the function atan2()

        return Z #Leave this line here. Return vector Z of 4 bearings.
    
    # --------
    # set_noise: 
    #	sets the noise parameters
    #

    def set_noise(self, new_b_noise, new_s_noise, new_d_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.bearing_noise  = float(new_b_noise)
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)
    
    ############# ONLY ADD/MODIFY CODE BELOW HERE ###################

    # --------
    # move:
    #   move along a section of a circular path according to motion
    #
    
    def move(self, motion): # Do not change the name of this function

        steering = motion[0]
        distance = motion[1]
        
        if abs(steering) > max_steering_angle:
            raise(ValueError, 'Excedding max steering angle')
        if distance < 0.0:
            raise(ValueError, 'Movng backwards is not valid')
            
        res = robot()
        res.length = self.length
        res.bearing_noise = self.bearing_noise
        res.steering_noise = self.steering_noise
        res.distance_noise = self.distance_noise
        
        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)
        
        # Execute motion
        turn = tan(steering2) * distance2 / res.length
        if abs(turn) < 0.001:
            
            # approximate by straight line motion
            res.x = self.x + (distance2*cos(self.orientation))
            res.y = self.y + (distance2*cos(self.orientation))
            res.orientation = (self.orientation + turn) % (2.0*pi)
        else:
            radius = distance2 / turn
            cx = self.x - (sin(self.orientation)*radius)
            cy = self.y + (cos(self.orientation)*radius)
            res.orientation = (self.orientation + turn) % (2.0*pi)
            res.x = cx + (sin(res.orientation)*radius)
            res.y = cy - (cos(res.orientation)*radius)
            
        return res # make sure your move function returns an instance
                      # of the robot class with the correct coordinates.
                      
    ############## ONLY ADD/MODIFY CODE ABOVE HERE ####################
        

length = 20.

motions = [[2. * pi / 10, 20.] for row in range(8)]
myrobot = robot(length)
myrobot.set(0, 0, 0)
myrobot.set_noise(bearing_noise, steering_noise, distance_noise)

for i in range(len(motions)):
    myrobot.move(motions[i])
#    print('Robot:        ', myrobot)
    print('Measurements: ', myrobot.sense())







