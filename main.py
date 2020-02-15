from environment import env

import visuals as vs
import drone as dr
import obstacle_grid as og
import hub as hb


env.grid = og.ObstacleGrid(env.t_simulation)

for i in range(3):
    env.hubs.append(hb.Hub())

""" Run RRT* on each hub graph """
for t in range(env.t_expand):
    for hub in env.hubs:
        hub.graph.newIteration()

    if t%100==0:
        vs.draw_scene(0, [])

import visuals as vs
import drone as dr
import pickle


"""Loading them back up again"""
with open('grid_file', 'rb') as grid_file:
    env.grid = pickle.load(grid_file)


for b in range(3):
    with open('hub_' + str(b), 'rb') as hub_file:
        env.hubs.append(pickle.load(hub_file))



""" Create a drone each 100 timesteps """
for t in range(env.t_simulation):
    if t % 20 == 0:
        env.drones.append(dr.Drone(t))

i = 0

""" Draw simulation """
while True:
    i += 1
    if i % 1 == 0:
        vs.draw_scene(i%env.t_simulation, [])  # Draw scene (at time t)

    # drones[0].node = Node(screen_to_cartesian(pygame.mouse.get_pos()),None)
    # drones[0].path = None
    # drones[0].check_path()
