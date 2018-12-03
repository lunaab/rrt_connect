import graph_tool.all as gs
import numpy as np
import sys
import pylab as pl
import time
from matplotlib import collections as mc
import matplotlib.pyplot as plt

''' This is a class that runs the PRM algorithm.
    Input:
        start: 2 tuple<float>
        goal: 2 tuple<float>
        dimensions: int
        k: int
        obstacles: list<3-tuple<float>>
        iters: int 
    Note: The actual path is not currently created by peforming graph search.
          The algorithm terminates when all of the nodes have been connected to their
          k nearest neighbors. '''

class ProbRoadMap(object):

    def __init__(self, start, goal, dimensions, obstacles, k, iters):
    
        # Input parameters
        self.start = start
        self.goal = goal
        
        self.dimensions = dimensions
        self.iters = iters
        self.k = k
        
        self.obstacles = obstacles
        
        # Stores connections and paths
        self.graph = gs.Graph(directed=False)
        self.vert_state = self.graph.new_vertex_property('vector<float>')
        
        # Add start and goal
        self.shadow_state = np.array([start]) # np array for faster computation
        vert = self.graph.add_vertex()
        self.vert_state[vert] = np.array(start)
        self.add_vertex(goal)
        
    def add_vertex(self, coords):
        ''' Adds a vertex to the graph when given the coordinates'''
        vert = self.graph.add_vertex()
        self.vert_state[vert] = np.array(coords)
        self.shadow_state = np.append(self.shadow_state, np.array([coords]),axis=0)
        return vert
        
    def build(self):
        ''' Runs the three main phases of the PRM '''
        self.place_vertices()
        self.local_planner()
        # self.find_path()
        
    def collides(self, q):
        ''' Checks to see if a point is within an obstacle '''
        for obs in self.obstacles:
            if np.sum((q - obs[0:self.dimensions])**2)**(0.5) <= obs[self.dimensions]:
                return True
        return False
        
    def can_connect(self, q1, q2):
        ''' Checks to see if the path connecting two points intersects an obstacle '''
        # Only works for 2D case currently
        samples = 1000
        
        # build a set of points along the line
        x = np.linspace(q1[0], q2[0], samples)
        m = (q1[1]-q2[1])/(q1[0]-q2[0])
        y = m*(x-q1[0]) + q1[1]
        points = np.append(x.reshape(samples,1), y.reshape(samples,1),axis=1)
        
        # check samples number of points along the line for collision
        for obs in self.obstacles:
            dists = np.sum((points - obs[0:self.dimensions])**2, axis=1)**(0.5)
            if np.min(dists) <= obs[self.dimensions]:
                return False
        return True
        
    def k_nearest_neighbors(self, q):
        ''' Determines the k nearest neighbors in the graph to q.
            Note that the neighbors are not given in any particular order . '''
        # q is an n-dimensional point
        dist = q - self.shadow_state
        dist = np.sum((dist**2), axis=1) # determine distance to each point 
        my_point = np.argwhere(dist==0) # idx of q
        ret = np.argpartition(dist.reshape(dist.shape[0],), self.k+1)[0:self.k+1] # nn
        my_idx = np.argwhere(ret==my_point) # idx of q in ret
        return np.delete(ret, [my_idx[0,1]]) # remove q itself from the list
        
    def local_planner(self):
        ''' Connects each point to its k-nn as long as the path
            does not go through an obstacle '''
            
        for k in range(0, self.shadow_state.shape[0]):
            q1 = self.shadow_state[k,:]
            neighbors = self.k_nearest_neighbors(q1)
            for j in range(0, neighbors.shape[0]):
                if self.can_connect(q1, self.shadow_state[neighbors[j],:]):
                    self.graph.add_edge(self.graph.vertex(k), self.graph.vertex(neighbors[j]))
        
    def place_vertices(self):
        ''' Randomly places k vertices in the space.
            If a vertex is placed within an obstacle it is deleted and
            a new vertex is placed. '''
        for k in range(0, self.iters):
            q_random = (np.random.rand(self.dimensions)-0.5)*20
            if not self.collides(q_random):
                self.add_vertex(q_random)
            else:
                k = k-1
                
    def visualize(self):
        ''' Draws the graph in 2D along with the circular obstacles '''
        lines = []
        for edge in self.graph.get_edges():
            p1 = (self.vert_state[self.graph.vertex(edge[0])][0], self.vert_state[self.graph.vertex(edge[0])][1])
            p2 = (self.vert_state[self.graph.vertex(edge[1])][0], self.vert_state[self.graph.vertex(edge[1])][1]) 
            lines.append([p1, p2])
        
        lc = mc.LineCollection(lines)
        fig, ax = pl.subplots()
        
        # Obstacles
        for circ in self.obstacles:
            circle = plt.Circle((circ[0], circ[1]), circ[2])
            ax.add_patch(circle)
        
        ax.add_collection(lc)
        ax.autoscale()
        fig.show()
        input('Press Enter to clear screen')
        
        
if __name__ == '__main__':
    start = (0,0)
    goal = (5,5)
    dimensions = 2
    k = 3
    iters = 100
    obstacles = [(4, 3, 2), (2, 1, 2)]
    test = ProbRoadMap(start, goal, dimensions, obstacles, k, iters)
    test.build()
    test.visualize()
