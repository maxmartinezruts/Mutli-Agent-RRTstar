import numpy as np
from copy import deepcopy
from environment import env as en

""" Convert a position given in cartesian coordinates to the coordinates of a (discrete) grid """
def cartesianToCoor(cartesian,sizeGrid):
   coor  = np.array([(cartesian[0]+8)/16*sizeGrid,(cartesian[1]+8)/16*sizeGrid]).astype(int)
   return np.array([min(max(coor[0],0),sizeGrid-1),min(max(coor[1],0), sizeGrid-1)])


""" Given the coordinates of a grid return the position in cartesian coordinates """
def coorToCartesian(coor,sizeGrid):
  cartesian = np.array([coor[0]*16/sizeGrid-8,coor[1]*16/sizeGrid-8]).astype(float)
  return cartesian


""" Generate a random point which is not colliding with any obstacle """
def random_pos_collision_free():
    obstacleFound = True
    while obstacleFound:                                                        # Repeat until obstacle not found
       x_random = np.random.uniform(-8, 8, 2)                                  # Generate random position
       x_random_coor = cartesianToCoor(x_random, en.grid.n)                       # Convert position to grid coordinates
       if len(en.grid.grid[0][x_random_coor[0]][x_random_coor[1]]) == 0:                # Check if cell on the grid is obstacle free
           obstacleFound = False
    return x_random

""" Generate a random point (in an ellipse) which is not colliding with any obstacle """
def random_pos_ellipse_collision_free(best_c, x_goal, x_start):

    min_path = np.linalg.norm(x_goal - x_start)     # Distance (on straight line) from goal to start)
    x_centre = (x_start + x_goal) / 2               # Point in between start and goal (centre of ellipse)

    """ Create random point in unit circle """
    length = np.sqrt(np.random.uniform(0, 1))   # Random length (sqrt to account for uniform random points in a circle)
    angle = np.pi * np.random.uniform(0, 2)     # Random angle
    x = length * np.cos(angle)
    y = length * np.sin(angle)
    x_disc = np.array([x, y])                   # Random point is an unit circle

    """ Create linear algebra transformations """
    L = np.array([[best_c / 2, 0], [0, np.sqrt(best_c ** 2 - min_path ** 2) / 2]])      # Scaling matrix transformation | scale #THIS IS NOT EXACTLY CORRECT BECAUSE BEST_C should NEVER BE SMALLER THAN MIN PATH, needs fix better than ABS()
    delta = np.arctan2(x_goal[1] - x_start[1], x_goal[0] - x_start[0])                  # Angle of the vector start->goal
    C = np.array([[np.cos(delta), -np.sin(delta)], [np.sin(delta), np.cos(delta)]])     # Rotation matrix transformation

    x_random = (C.dot(L)).dot(x_disc) + x_centre                                        # Scale, then rotate, then shift

    return x_random

class CBS:
    def __init__(self, drones):

        self.nodes = []
        for drone in drones:
            drone.graph = deepcopy(drone.hub.graph)
            drone.graph.t_start = drone.t_start

        collisions = True
        while collisions:
            paths = []
            for drone in drones:
                paths.append(drone.check_path)

            t = 0
            while t < en.t_simulation:
                t+=1
                for drone in drones:
                    for obstacle in drones:
                        if drone != obstacle and np.linalg.norm(drone.node.pos - obstacle.node.pos) < 1:
                            print("Collisiont", drone, obstacle)

            # for i in range(1000):
            #     drone.graph.newIteration()
            #     drone.check_path()
            #     if i % 100 == 0:
            #         draw_scene(t, [drone])
            #         print(drone.graph.edges)
            # drone.create_trace()

    class Node:
        def __init__(self):
            self.score = 0




