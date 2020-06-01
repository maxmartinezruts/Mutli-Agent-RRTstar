import random
import RRT_star as rrt
from environment import env as en
import helpers as hp
import numpy as np
class Drone:

    v = 0.05

    def __init__(self, t_start):
        self.t_start = t_start
        self.hub = random.choice(en.hubs)
        self.path = None
        self.node = rrt.Node(hp.random_pos_collision_free(),None)
        self.trace = []
        self.check_path()
        self.create_trace()


    def check_path(self):
        """ If n_new is close to drone, create path and if cost of path is lower than lowest yet found, set path as new path of drone """

        for n_new  in self.hub.graph.nodes:
            if np.linalg.norm(n_new.pos - self.node.pos) < 0.5:
                path = rrt.Path(n_new, self.hub.graph.nodes[0], self)
                if self.path == None or path.cost < self.path.cost:
                    self.path = path

                    return  path
        return None

    def create_trace(self):
        if self.path == None:
            return 'No path'
        path = self.path.edges[::-1]

        i = 0
        dis_covered = 0
        while i < len(path):
            edge = path[i]
            pos = edge.st.pos + edge.vec/edge.len*dis_covered

            self.trace.append(pos)
            if dis_covered + self.v < edge.len:
                dis_covered += self.v
            else:
                i += 1
                dis_covered = dis_covered + self.v - edge.len

        self.add_trace_to_grid(self.trace, self.t_start)

    def check_collisions(self):
        for node in self.hub.graph.nodes:
            if len(grid.grid[min(int(node.cost/self.v)+self.t_start, t_simulation-1)][node.coor_obs[0]][node.coor_obs[1]]):
                self.hub.graph.delete_tree(node)

    def add_trace_to_grid(self, trace, t_start):

        """ Given a trace and the starting time add the obstacle cells to the obstacle grid """

        t = t_start
        for point in trace:
            t += 1
            coor = hp.cartesianToCoor(point, en.grid.n)
            if t < en.t_simulation:
                neighbours = [[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1], [0, 0]]
                for neighbour in neighbours:
                    coor_neighbour = [coor[0] + neighbour[0], coor[1] + neighbour[1]]
                    if 0 <= coor_neighbour[0] < en.grid.n and 0 <= coor_neighbour[1] < en.grid.n:
                        for obstacle in en.grid.grid[t][coor_neighbour[0]][coor_neighbour[1]]:
                            # print("collision between ", obstacle, "and ", drone)
                            if obstacle != True and "collision" not in en.grid.grid[t][coor_neighbour[0]][
                                coor_neighbour[1]]:
                                en.grid.grid[t][coor_neighbour[0]][coor_neighbour[1]].append('collision')
                        if self not in en.grid.grid[t][coor_neighbour[0]][coor_neighbour[1]]:
                            en.grid.grid[t][coor_neighbour[0]][coor_neighbour[1]].append(self)

