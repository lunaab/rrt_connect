import graph_tool.all as gs
import numpy as np
import sys
import pylab as pl
from matplotlib import collections as mc


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
        self.add_vertex(np.array(start))
        self.build()
        
    def add_vertex(self, coords):
        vert = self.tree.add_vertex()
        self.vert_state[vert] = np.array(coords)
        return vert
        
    def build(self):
        for k in range(0, iters):
            q_random = (np.random.rand(self.dimensions)-0.5)*15
            #print q_random
            self.extend(q_random)
            
    def collides(self, q):
        for obs in self.obstacles:
            if np.sum((q - obs[0:dimensions])**2)**(0.5) <= obs[dimensions]:
                return False
        return True
               
    def extend(self, q):
        q_near = self.nearest_neighbor(q) #index
        #print q_near
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
        diff = q - self.vert_state[self.tree.vertex(q_near)]
        delta = self.epsilon * diff / np.sum((diff)**2)**(0.5)
        #print delta
        return self.vert_state[self.tree.vertex(q_near)] + delta
        
        
    def nearest_neighbor(self, q):
        nearest = 0;
        min_dist = sys.maxint;
        for vert in self.tree.get_vertices():
            dist = np.sum((q - self.vert_state[vert])**2)**(0.5)
            if dist < min_dist:
                nearest = vert
                min_dist = dist
        return nearest
        
    def new_config(self, q_new):
        return self.collides(q_new)
            
        
    def visualize(self):
        lines = []
        for edge in self.tree.get_edges():
            p1 = (self.vert_state[self.tree.vertex(edge[0])][0], self.vert_state[self.tree.vertex(edge[0])][1])
            p2 = (self.vert_state[self.tree.vertex(edge[1])][0], self.vert_state[self.tree.vertex(edge[1])][1]) 
            lines.append([p1, p2])
            
        lc = mc.LineCollection(lines)
        fig, ax = pl.subplots()
        ax.add_collection(lc)
        ax.autoscale()
        fig.show()
        input('Press Enter to clear screen')


if __name__ == '__main__':
    start = (0,0)
    goal = (5,5)
    dimensions = 2
    epsilon = 0.1
    iters = 500
    obstacles = [(4, 3, 0.5), (2, 1, 0.2)]
    test = RRT(start, goal, dimensions, epsilon, obstacles, iters)
    test.visualize()
