import graph_tool.all as gs
import numpy as np
import sys
import pylab as pl
import time
from matplotlib import collections as mc
import matplotlib.pyplot as plt
from rrt_connect import RRT
from prob_rm import ProbRoadMap


''' This code is for running the empirical experiments.
    PRM and RRT connect are tasked to find a path in the same
    evironment. The maximum number of nodes they are allowed to use varies.
    The execution time is currently averaged over 5 trials. '''

# Constants
num_avg = 5
start = (0,0)
goal = (7,7)
dimensions = 2
obstacles = [(4, 3, 2), (2, 1, 2)]
k = 3
epsilon = 0.5

iters = [100, 200, 300, 400, 500, 600, 700]

# Timing
road_map_time = []
rrt_time = []

# Looping through different maximum numbers of nodes
for j in range(0, len(iters)):

    # Adding a new timing element for each algorithm
    road_map_time.append(0)
    rrt_time.append(0)
    
    # Executing the planning multiple times and averaging the time
    for i in range(0, num_avg):
        start_time = time.time()
        rm = ProbRoadMap(start, goal, dimensions, obstacles, k, iters[j])
        rm.build()
        road_map_time[j] = road_map_time[j] + (1.0/num_avg)*(time.time() - start_time)

        start_time = time.time()
        rrt = RRT(start, goal, dimensions, epsilon, obstacles, iters[j])
        rrt.connect_planner()
        rrt_time[j] = rrt_time[j] + (1.0/num_avg)*(time.time() - start_time)

# Plotting the Results
plt.plot(iters, road_map_time, label='PRM')
plt.plot(iters, rrt_time, label='RRT')
plt.xlabel('Maximum number of nodes')
plt.ylabel('Time (s)')
plt.legend()
plt.show()
input('Press Enter to Clear')

print road_map_time
print rrt_time
