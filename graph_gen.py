import pickle
from environment import env
import obstacle_grid as og
import hub as hb


""" Run RRT* on each hub graph """
env.grid = og.ObstacleGrid(env.t_simulation)

num_hubs = 3



for i in range(num_hubs):
   env.hubs.append(hb.Hub())

for t in range(env.t_expand):
    for hub in env.hubs:
        hub.graph.newIteration()



"""Saving hubs and grid"""
for a in range(len(env.hubs)):
    saving_hub = env.hubs[a]
    with open('hub_' + str(a), 'wb') as hub_file:
        pickle.dump(saving_hub,hub_file)

with open('grid_file','wb') as grid_file:
    pickle.dump(env.grid,grid_file)


