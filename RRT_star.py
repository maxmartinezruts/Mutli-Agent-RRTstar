import helpers as hp
from  environment import env as en
import numpy as np
import math

class Node:
    """ Set position and parent attributes, then if node has parent, its cost is parent's cost + distance to parent """
    def __init__(self, pos, parent):
        self.pos = pos                              # Node position
        self.coor = hp.cartesianToCoor(self.pos, 50)
        self.coor_obs = hp.cartesianToCoor(self.pos, en.grid.n)
        self.parent = parent                        # Node parent
        self.e_in = None
        if parent == None:
            self.cost = 0                           # If no parent (node is root) cost is 0
        else:
            self.cost = parent.cost + np.linalg.norm(pos-parent.pos)            # Cost of node (from root to node)

        self.children = []

class Edge:

    """ Set start node, end node and cost instance attributes """
    def __init__(self, st, en, graph):
        self.st = st                                # Start node of edge
        self.en = en                                # End node of edge
        self.len = np.linalg.norm(st.pos-en.pos)
        self.cost = self.len                        # Cost of edge is its length
        self.vec = self.en.pos - self.st.pos        # Edge vector
        self.t_start = graph.t_start + st.cost
        self.graph = graph

    """ Check if edge is colliding with an obstacle (by checking cells where the edge is present) """
    def is_colliding(self):
        colliding = False
        cuts = np.linspace(0, 1, 10)  # Cut unit in 10 uniform intervals
        for cut in cuts:
            cut_pos = (self.en.pos - self.st.pos) * cut + self.st.pos  # Position of cut
            coor = hp.cartesianToCoor(cut_pos, en.grid.n)  # Grid coordinates of cut
            t = min(self.graph.t_start + int(self.st.cost + self.len/0.05 * cut), en.t_simulation-1)
            r = 1
            for i in range(min(max(-r+coor[0],0),100),min(max(r+1+coor[0],0),100)):
                for j in range(min(max(-r+coor[1],0),100),min(max(r+1+coor[1],0),100)):
                    colliding = len(en.grid.grid[t][i][j]) or colliding  # Set colliding = True if cell on the grid has obstacle
        return colliding

class Path:
    """ Create a path instance given the root, the leaf and the goal nodes """
    def __init__(self, leaf, root, drone):
        self.cost = 0
        self.edges = []                                         # Start empty list of edges


        edge_drone_leaf = Edge(leaf, drone.node, drone.graph)
        self.edges.append(edge_drone_leaf)
        self.cost += edge_drone_leaf.cost
        node = leaf                                             # Start from the leaf
        while node != root:                                     # While root is not reached
            edge = node.e_in                                    # New edge is predecessor (e_in) of previous edge
            self.edges.append(edge)                             # Add new edge to the path
            self.cost += edge.cost                              # Add cost to total cost of the path
            node = node.parent                                  # New node is previous node's parent

        # self.cost += np.linalg.norm(leaf.pos - goal.pos)        # Leaf post != Goal post, so add the distance between goal and leaf to the cost of the path

class Graph:

    n = 50
    """ Create a graph by ... """
    def __init__(self, root, t_start):
        self.nodes = []
        self.edges = []
        self.nodes.append(root)
        self.localizer_grid = [[[] for _ in range(self.n)]for _ in range(self.n)]
        hub_coor = hp.cartesianToCoor(root.pos, self.n)
        self.localizer_grid[hub_coor[0]][hub_coor[1]].append(root)
        self.t_start = t_start

    def newIteration(self):

        step_range = 0.5
        neighbor_range = 1

        if True:                                                        # If a path HAS NOT been yet found, then just create a random point (RRT*)
            x_random = hp.random_pos_collision_free()
        else:                                                           # If a path HAS been found already, then create random point within an ellipse (informed RRT*)
            x_random = hp.random_pos_ellipse_collision_free(self.path.cost, self.goal.pos, self.start.pos)

        coor = hp.cartesianToCoor(x_random, 50)
        """ Look for the nearest node to x_random """
        r = 0
        found = []
        min_dist = math.inf
        while len(found) == 0:
            for i in range(min(max(-r+coor[0],0),50),min(max(r+1+coor[0],0),50)):
                for j in range(min(max(-r+coor[1],0),50),min(max(r+1+coor[1],0),50)):
                    for node in self.localizer_grid[i][j]:
                        found.append(node)
                        if np.linalg.norm(node.pos -x_random) < min_dist:
                            n_nearest = node
                            min_dist = np.linalg.norm(node.pos -x_random)

            r+=1




        """ Create a point close to n_nearest on the direction of x_random """
        x_new = (x_random-n_nearest.pos)/np.linalg.norm(n_nearest.pos - x_random)*step_range + n_nearest.pos
        x_new_coor = hp.cartesianToCoor(x_new, self.n)
        """ Define a set of nodes on the neighbourhood of x_new """
        N_near = []
        r= 5
        for i in range(min(max(-r+x_new_coor[0],0),self.n),min(max(r+1+x_new_coor[0],0),self.n)):
            for j in range(min(max(-r + x_new_coor[1], 0), self.n), min(max(r+1 + x_new_coor[1], 0), self.n)):
                for node in self.localizer_grid[i][j]:
                    if np.linalg.norm(node.pos - x_new) < neighbor_range:
                        N_near.append(node)


        """ Find for which node in N_near the path to root would be the least """
        n_min = n_nearest
        c_min = n_nearest.cost + np.linalg.norm(n_nearest.pos - x_new)
        for n_near in N_near:
            cost = n_near.cost + np.linalg.norm(n_near.pos-x_new)
            if cost < c_min:
                c_min = cost
                n_min = n_near

        """ Create new node at x_new with parent n_min and an edge connecting parent and n_new """
        n_new = Node(x_new, n_min)
        e_new = Edge(n_min, n_new, self)
        # n_new.e_in = e_new

        """ If the new edge is not colliding with any obstacle in the grid, then add it to the graph """
        if not e_new.is_colliding():
            self.nodes.append(n_new)
            new_coor = hp.cartesianToCoor(n_new.pos, self.n)
            self.localizer_grid[new_coor[0]][new_coor[1]].append(n_new)
            self.edges.append(e_new)
            n_new.parent.children.append(n_new)
            n_new.e_in = e_new

            """ RRT* | erase inefficient connections and create shorter connections in the neighbourhood of x_new """
            for n_near in N_near:
                cost = n_new.cost + np.linalg.norm(n_new.pos - n_near.pos)
                if cost < n_near.cost:
                    e_new = Edge(n_new, n_near, self)
                    if not e_new.is_colliding():
                        n_near.parent.children.remove(n_near)
                        n_near.parent = n_new
                        n_new.children.append(n_near)
                        n_near.cost = cost
                        self.edges.remove(n_near.e_in)
                        n_near.e_in = e_new
                        self.edges.append(e_new)

    def check_collision(self):
        for i in range(20,30):
            for j in range(20,30):
                for node in self.localizer_grid[i][j]:
                    self.delete_tree(node)
        return 0

    def delete_tree(self, node):
        if node.parent != None:
            node.parent.children.remove(node)

        self.nodes.remove(node)
        self.localizer_grid[node.coor[0]][node.coor[1]].remove(node)
        self.edges.remove(node.e_in)
        copy_list = list(node.children)
        for child in copy_list:
            self.delete_tree(child)

