import graph_tool.all as gs
import numpy as np
import sys
import pylab as pl
import time
from matplotlib import collections as mc
import matplotlib.pyplot as plt


''' This is a class that runs the RRT-connect algorithm.
    Input:
        start: 2 tuple<float>
        goal: 2 tuple<float>
        dimensions: int
        epsilon: float
        obstacles: list<3-tuple<float>>
        iters: int 
    Note: The actual path is not currently created by backing out from the connection.
          The algorithm terminates when a connection is made or the maximum iterations have
          been achieved. '''


class RRT(object):

    def __init__(self, start, goal, dimensions, epsilon, obstacles, iters):
    
        # Constants
        self.extend_ret = {'Reached': 0, 'Advanced':1, 'Trapped':2}
        #self.fig, self.ax = pl.subplots()
    
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
        self.shadow_1 = np.array([start]) #np array of coords for faster processing
        vert = self.tree_1.add_vertex()
        self.vstate_1[vert] = np.array(start) #additional array of coords
        
        # Tree at goal
        self.tree_2 = gs.Graph(directed=False)
        self.vstate_2 = self.tree_2.new_vertex_property('vector<float>')
        self.shadow_2 = np.array([goal])
        vert = self.tree_2.add_vertex()
        self.vstate_2[vert] = np.array(goal)
        
    def add_vertex(self, coords, which):
        ''' Adds a vertex to a tree when given the coordinates and tree id '''
        
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
        ''' Iterates through maximum number of nodes.
            A random node is selected. Then we extend the current first tree.
            We then tree to connect the second tree. '''
            
        for k in range(0, self.iters):
            q_random = (np.random.rand(self.dimensions)-0.5)*20 # Random point in space
            output = self.extend(q_random, self.my_trees[0]) # Extend tree
            if output[0] is not self.extend_ret['Trapped']:
                # Connect tree if we made a new node and return if path found 
                if self.connect(output[1], self.my_trees[1]) == self.extend_ret['Reached']:
                    #self.find_path()
                    return
            self.swap()
            
    def connect(self, q, which):
        ''' Extends the tree toward q until it becomes trapped or 
            reaches q. '''
            
        s = self.extend_ret['Advanced']
        while s == self.extend_ret['Advanced']:
            s = self.extend(q, which)[0]
        return s
            
    def collides(self, q):
        ''' Tests if a point q lies within an obstacle '''
        for obs in self.obstacles:
            if np.sum((q - obs[0:self.dimensions])**2)**(0.5) <= obs[self.dimensions]:
                return True
        return False
               
    def extend(self, q, which):
        ''' Finds the nearest neighbor in the selected graph to q.
            Then attempts to create a new point toward q that is connected
            to the nearest neighbor '''
            
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
        ''' Creates a new point atleast epsilon from q_near toward q in the
            selected tree '''
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
        ''' Finds the nearest neighbor of a point q on a specified graph '''
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
        ''' A point is a new configuration if it does not hit an obstacle '''
        return not self.collides(q_new)
        
    def swap(self):
        ''' Swaps the order of the trees in the tuple.'''
        self.my_trees = (self.my_trees[1], self.my_trees[0])
            
        
    def visualize(self):
        ''' Draws  trees in 2D along with circular obstacles'''
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
        
        self.fig, self.ax = pl.subplots()
        self.ax.clear()
        
        # Obstacles
        for circ in self.obstacles:
            circle = plt.Circle((circ[0], circ[1]), circ[2])
            self.ax.add_patch(circle)
        
        
        self.ax.add_collection(lc1)
        self.ax.add_collection(lc2)
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        #ax.autoscale()
        self.fig.canvas.draw()
        self.fig.show()
        
        
        input('Press Enter to clear screen')


if __name__ == '__main__':
    start = (0,0)
    goal = (7,7)
    dimensions = 2
    epsilon = 0.5
    iters = 1000
    obstacles = [(4, 4, 2), (4, 1, 2)]
    test = RRT(start, goal, dimensions, epsilon, obstacles, iters)
    test.connect_planner()
