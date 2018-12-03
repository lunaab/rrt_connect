import graph_tool.all as gs
import numpy as np
import sys
import pylab as pl
import time
from matplotlib import collections as mc
import matplotlib.pyplot as plt


class RRT(object):

    def __init__(self, start, goal, dimensions, epsilon, obstacles, iters):
    
        # Constants
        self.extend_ret = {'Reached': 0, 'Advanced':1, 'Trapped':2}
    
        # Input parameters
        self.start = start
        self.goal = goal
        
        self.epsilon = epsilon
        self.dimensions = dimensions
        self.iters = iters
        
        self.obstacles = obstacles
        
        # Stores connections and paths
        self.tree = gs.Graph(directed=False)
        self.vert_state = self.tree.new_vertex_property('vector<float>')
        self.shadow_state = np.array([start])
        vert = self.tree.add_vertex()
        self.vert_state[vert] = np.array(start)
        
    def add_vertex(self, coords):
        ''' Adds a vertex to the tree given its coords'''
        
        vert = self.tree.add_vertex()
        self.vert_state[vert] = np.array(coords)
        self.shadow_state = np.append(self.shadow_state, np.array([coords]),axis=0)
        return vert
        
    def build(self):
        ''' Builds the tree over iters iterations '''
        
        for k in range(0, iters):
            q_random = (np.random.rand(self.dimensions)-0.5)*20
            self.extend(q_random)
            
    def collides(self, q):
        ''' Tests if a point is within an obstacle '''
        
        for obs in self.obstacles:
            if np.sum((q - obs[0:dimensions])**2)**(0.5) <= obs[dimensions]:
                return True
        return False
               
    def extend(self, q):
        ''' Creates a new point toward q from its nearest neighbor on
            the tree. Will connect to q if it is less than epsilon away. '''
            
        q_near = self.nearest_neighbor(q) #index
        q_new = self.move_to_q(q_near,q) #(1,...,n) point
        if (self.new_config(q_new)):
            v_new = self.add_vertex(q_new)
            self.tree.add_edge(v_new, self.tree.vertex(q_near))
            
            if np.equal(q_new, q).mean() == 1.0:
                return self.extend_ret['Reached']
            else:
                return self.extend_ret['Advanced']
                
        return self.extend_ret['Trapped']
        
    def move_to_q(self, q_near, q):
        ''' Creates a new point toward q from its nearest neighbor on
            the tree. Will connect to q if it is less than epsilon away. '''
        diff = q - self.vert_state[self.tree.vertex(q_near)]
        dist = np.sum((diff)**2)**(0.5)
        if dist <= self.epsilon:
            return q
        else:
            delta = self.epsilon * diff / dist
            return self.vert_state[self.tree.vertex(q_near)] + delta
        
        
    def nearest_neighbor(self, q):
        ''' Finds the nearest neighbor to q in the tree '''
        dist = q - self.shadow_state
        dist = np.sum((dist**2), axis=1)
        nearest = np.argmin(dist)
    
        #nearest = 0;
        #min_dist = sys.maxint;
        #for vert in self.tree.get_vertices():
        #    dist = np.sum((q - self.vert_state[vert])**2)**(0.5)
        #    if dist < min_dist:
        #        nearest = vert
        #        min_dist = dist
        return nearest
        
    def new_config(self, q_new):
        ''' A new configuration is one that is not within an obstacle '''
        return not self.collides(q_new)
            
        
    def visualize(self):
        ''' Draws the tree in 2D with obstacles '''
        lines = []
        for edge in self.tree.get_edges():
            p1 = (self.vert_state[self.tree.vertex(edge[0])][0], self.vert_state[self.tree.vertex(edge[0])][1])
            p2 = (self.vert_state[self.tree.vertex(edge[1])][0], self.vert_state[self.tree.vertex(edge[1])][1]) 
            lines.append([p1, p2])
            
        lc = mc.LineCollection(lines)
        fig, ax = pl.subplots()
        
        # Obstacles
        for circ in obstacles:
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
    epsilon = 0.1
    iters = 10000
    obstacles = [(4, 3, 2), (2, 1, 2)]
    test = RRT(start, goal, dimensions, epsilon, obstacles, iters)
    test.build()
    print 'Starting Vis'
    test.visualize()
