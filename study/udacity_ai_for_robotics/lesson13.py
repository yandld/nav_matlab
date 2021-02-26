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

# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space


grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid, init, goal, cost):
    closed = [[0 for row in range(len(grid[0]))] for rol in range(len(grid))]
    expand = [[-1 for row in range(len(grid[0]))] for rol in range(len(grid))]
    action = [[-1 for row in range(len(grid[0]))] for rol in range(len(grid))]
    closed[init[0]][init[0]] = 1

    x = init[0]
    y = init[1]
    g = 0
    count = 0
    path = 0
    
    open = [[g, x, y]]
    
#    for i in range(len(open)):
#        print('o:    ', open[i])
    
    found = False # flag that is set when serach complete
    resign = False # flasg set if we cann't find expand
    
    while not found and not resign:
        # check if we still have elements on the open list
        if(len(open) == 0):
            resign = True
            print('find fail')
        else:
            open.sort()
            open.reverse()
            next = open.pop()
#            print('next:', next)
            x = next[1]
            y = next[2]
            g = next[0]
            expand[x][y] = count
            count += 1
        
        if x == goal[0] and y == goal[1]:
            found = True
            print('we found the target')
            path = [g, x, y]
        else:
            for i in range(len(delta)):
                x2 = x + delta[i][0]
                y2 = y + delta[i][1]   
                if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
                    if closed[x2][y2] == 0 and grid[x2][y2] == 0:                        
                        g2 = g + cost
                        open.append([g2, x2, y2])
                        #print('append:', [g2, x2, y2])
                        closed[x2][y2] = 1
                        action[x2][y2] = i

    policy = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    x = goal[0]
    y = goal[1]
    policy[x][y] = '*'
    while x != init[0] or y != init[1]:
        x2 = x - delta[action[x][y]][0]
        y2 = y - delta[action[x][y]][1]
        policy[x2][y2] = delta_name[action[x][y]]
        x = x2
        y = y2


    print('path:', [g, x, y])
    print('expand:', expand)
    print('action:', action)
    print('policy:', policy)
    
    return path

path = search(grid, init, goal ,cost)
#print(path)
