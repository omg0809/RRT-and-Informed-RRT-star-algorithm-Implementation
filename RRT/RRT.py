# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
import math

# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        return np.sqrt((node1.row-node2.row)**2+(node1.col-node2.col)**2)

    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        
        N=40
        x_d = (node1.row-node2.row)
        y_d = (node1.col-node2.col)

        x_coordinate = node2.row
        y_coordinate = node2.col     
        for i in range(N+1):
            
            if(self.map_array[int(x_coordinate)][int(y_coordinate)]==0): #0 is obstacle 
                return False
            x_coordinate=node2.row + x_d*(i/N)
            
            y_coordinate=node2.col + y_d*(i/N)
        return True


    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###
        goal_bias = 0.025
        rand_prob = np.random.random()
        if rand_prob<goal_bias:
            new_point = Node(self.goal.row,self.goal.col)
        else:
            new_point = Node(np.random.randint(0,self.size_row), np.random.randint(0,self.size_col))
        return new_point

    
    def get_nearest_node(self, new_point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        dst = 10000
        for node in self.vertices:
            if(self.dis(node,new_point)<dst):
                dst = self.dis(node,new_point)
                nearest_node = node
        
        return nearest_node
    
   

    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        neighbor_size = 20
        neighbors = []
        
        for node in self.vertices:
            if node == new_node:
                continue
            if self.dis(new_node, node)<neighbor_size:
                neighbors.append(node)

        return neighbors

    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###
        # neighbors = self.get_neighbors(new_node, neighbor_size=20)
        for neighbor_node in neighbors:  
            if neighbor_node == new_node.parent:
                continue
            if self.check_collision(neighbor_node,new_node):
                ct_new = new_node.cost + self.dis(new_node, neighbor_node)
                if ct_new < neighbor_node.cost:
                    neighbor_node.parent = new_node
                    neighbor_node.cost = ct_new
                    


    
    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()

    def extend_node(self,new_point,nearest_node):
        step_size = 16
        #getting the x,y coordinates of the node to make an extension
        x_new = new_point.row
        y_new = new_point.col

        x_near = nearest_node.row
        y_near = nearest_node.col

        #finding the distance between the new node and nearest node
        dist = self.dis(new_point,nearest_node)
        v = [x_new - x_near, y_new - y_near]
        new_pos_x = int(x_near+((step_size*v[0])/dist))
        new_pos_y = int(y_near+((step_size*v[1])/dist))
        
        #after finding the new position checking if it fits in the bounds of the map
        if(new_pos_x>=self.size_row):
            new_pos_x = self.size_row-1
        if(new_pos_y>=self.size_col):
            new_pos_y = self.size_col-1
        #checking for the edge cases
        if(new_pos_x<0):
            new_pos_x = 0
        if(new_pos_y<0):
            new_pos_y = 0
        
        #new pos is the extended point which is converted to the node
        new_pos = Node(new_pos_x,new_pos_y)
        
        return new_pos

                
    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.
        for i in range(n_pts):
            #get a new point
            new_point = self.get_new_point(goal_bias=0.05)
            
            #get its nearest node
            nearest_node = self.get_nearest_node(new_point)
            
    
            if self.dis(new_point,nearest_node)>12:
            #extend the node and check collision to decide whether to add or drop
                step_node = self.extend_node(new_point,nearest_node)
                new_point = step_node
                
            if(self.check_collision(new_point,nearest_node)):
                new_point.parent = nearest_node
                new_point.cost = nearest_node.cost + self.dis(new_point, nearest_node)
                self.vertices.append(new_point)  
                if new_point == self.goal:
                    self.found = True
            #if added, check if reach the neighbor region of the goal.
            if((self.dis(new_point, self.goal) <=2) and self.check_collision(new_point, self.goal)):
                self.found = True
                self.goal.parent = new_point
                self.goal.cost = new_point.cost + self.dis(new_point, self.goal)
                break 



        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.

        # Output
        for i in range(n_pts):
            #get a new point
            new_point = self.get_new_point(goal_bias=0.05)

            #get its nearest node
            nearest_node = self.get_nearest_node(new_point)

            if self.dis(new_point,nearest_node)>16:
            #extend the node and check collision to decide whether to add or drop
                step_node = self.extend_node(new_point,nearest_node)
                new_point = step_node

            if(self.check_collision(new_point,nearest_node)):
                new_point.parent = nearest_node
                new_point.cost = nearest_node.cost + self.dis(new_point, nearest_node)
                self.vertices.append(new_point)
                neighbors = self.get_neighbors(new_point, neighbor_size=20)
                for neighbor_node in neighbors:  
                    if neighbor_node == new_point.parent:
                        continue

                    if self.check_collision(neighbor_node,new_point):
                        new_cost = neighbor_node.cost + self.dis(new_point, neighbor_node)

                        if new_cost < new_point.cost:
                            new_point.parent = neighbor_node
                            new_point.cost = new_cost
                self.rewire(new_point,neighbors)
                
                if new_point == self.goal:
                    self.found = True
                
            if((self.dis(new_point, self.goal) <= 2) and self.check_collision(new_point, self.goal)):
                self.found = True
                self.goal.parent = new_point
                self.goal.cost = new_point.cost + self.dis(new_point, self.goal)        


        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
