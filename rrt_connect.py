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
        
        #tree pair indicator
        self.my_trees = (0, 1)
        
        # Tree at start
        self.tree_1 = gs.Graph(directed=False)
        self.vstate_1 = self.tree_1.new_vertex_property('vector<float>')
        self.shadow_1 = np.array([start])
        vert = self.tree_1.add_vertex()
        self.vstate_1[vert] = np.array(start)
        
        # Tree at goal
        self.tree_2 = gs.Graph(directed=False)
        self.vstate_2 = self.tree_2.new_vertex_property('vector<float>')
        self.shadow_2 = np.array([goal])
        vert = self.tree_2.add_vertex()
        self.vstate_2[vert] = np.array(goal)
        
    def add_vertex(self, coords, which):
        if which == 0:
            vert = self.tree_1.add_vertex()
            self.vstate_1[vert] = np.array(coords)
            self.shadow_1 = np.append(self.shadow_1, np.array([coords]),axis=0)
            return vert
        else:
            vert = self.tree_2.add_vertex()
            self.vstate_2[vert] = np.array(coords)
            self.shadow_2 = np.append(self.shadow_2, np.array([coords]),axis=0)
            return vert
            
        
    def connect_planner(self):
        for k in range(0, iters):
            q_random = (np.random.rand(self.dimensions)-0.5)*20
            output = self.extend(q_random, self.my_trees[0])
            if output[0] is not self.extend_ret['Trapped']:
                if self.connect(output[1], self.my_trees[1]) == self.extend_ret['Reached']:
                    print 'Path found'
                    return
            self.swap()
            
    def connect(self, q, which):
        s = self.extend_ret['Advanced']
        while s == self.extend_ret['Advanced']:
            s = self.extend(q, which)[0]
        return s
            
    def collides(self, q):
        for obs in self.obstacles:
            if np.sum((q - obs[0:dimensions])**2)**(0.5) <= obs[dimensions]:
                return False
        return True
               
    def extend(self, q, which):
        if which == 0:
            q_near = self.nearest_neighbor(q, which) #index
            q_new = self.move_to_q(q_near,q, which) #(1,...,n) point
            if (self.new_config(q_new)):
                v_new = self.add_vertex(q_new, which)
                self.tree_1.add_edge(v_new, self.tree_1.vertex(q_near))
            
                if np.equal(q_new, q).mean() == 1.0:
                    return (self.extend_ret['Reached'], q_new)
                else:
                    return (self.extend_ret['Advanced'], q_new)
                
            return (self.extend_ret['Trapped'], q_new)
        else:
            q_near = self.nearest_neighbor(q, which) #index
            q_new = self.move_to_q(q_near,q, which) #(1,...,n) point
            if (self.new_config(q_new)):
                v_new = self.add_vertex(q_new, which)
                self.tree_2.add_edge(v_new, self.tree_2.vertex(q_near))
            
                if np.equal(q_new, q).mean() == 1.0:
                    return (self.extend_ret['Reached'], q_new)
                else:
                    return (self.extend_ret['Advanced'], q_new)
                
            return (self.extend_ret['Trapped'], q_new)
        
    def move_to_q(self, q_near, q, which):
        if which == 0:
            diff = q - self.vstate_1[self.tree_1.vertex(q_near)]
            dist = np.sum((diff)**2)**(0.5)
            if dist <= self.epsilon:
                return q
            else:
                delta = self.epsilon * diff / dist
                return self.vstate_1[self.tree_1.vertex(q_near)] + delta
        else:
            diff = q - self.vstate_2[self.tree_2.vertex(q_near)]
            dist = np.sum((diff)**2)**(0.5)
            if dist <= self.epsilon:
                return q
            else:
                delta = self.epsilon * diff / dist
                return self.vstate_2[self.tree_2.vertex(q_near)] + delta
        
        
    def nearest_neighbor(self, q, which):
        if which == 0:
            dist = q - self.shadow_1
            dist = np.sum((dist**2), axis=1)
            nearest = np.argmin(dist)
            return nearest
        else:
            dist = q - self.shadow_2
            dist = np.sum((dist**2), axis=1)
            nearest = np.argmin(dist)
            return nearest
        
    def new_config(self, q_new):
        return self.collides(q_new)
        
    def swap(self):
        self.my_trees = (self.my_trees[1], self.my_trees[0])
            
        
    def visualize(self):
        # Tree One
        lines1 = []
        for edge in self.tree_1.get_edges():
            p1 = (self.vstate_1[self.tree_1.vertex(edge[0])][0], self.vstate_1[self.tree_1.vertex(edge[0])][1])
            p2 = (self.vstate_1[self.tree_1.vertex(edge[1])][0], self.vstate_1[self.tree_1.vertex(edge[1])][1]) 
            lines1.append([p1, p2])
            
        lc1 = mc.LineCollection(lines1, color=(1,0,1,1))
        
        # Tree Two
        lines2 = []
        for edge in self.tree_2.get_edges():
            p1 = (self.vstate_2[self.tree_2.vertex(edge[0])][0], self.vstate_2[self.tree_2.vertex(edge[0])][1])
            p2 = (self.vstate_2[self.tree_2.vertex(edge[1])][0], self.vstate_2[self.tree_2.vertex(edge[1])][1]) 
            lines2.append([p1, p2])
            
        lc2 = mc.LineCollection(lines2, color=(1,0,0,1))
        
        fig, ax = pl.subplots()
        
        # Obstacles
        for circ in obstacles:
            circle = plt.Circle((circ[0], circ[1]), circ[2])
            ax.add_patch(circle)
        
        
        ax.add_collection(lc1)
        ax.add_collection(lc2)
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        #ax.autoscale()
        fig.show()
        input('Press Enter to clear screen')


if __name__ == '__main__':
    start = (0,0)
    goal = (7,7)
    dimensions = 2
    epsilon = 0.1
    iters = 5000
    obstacles = [(4, 4, 2), (4, 1, 2)]
    test = RRT(start, goal, dimensions, epsilon, obstacles, iters)
    test.connect_planner()
    print 'Starting Vis'
    test.visualize()
