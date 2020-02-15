import random
import numpy as np
class ObstacleGrid:

    density = 0.1
    n = 100
    sparsity = 0.995

    def __init__(self, t_simulation):
        self.t_simulation = t_simulation
        self.grid = [[[[] for j in range(self.n)] for i in range(self.n)] for t in range(self.t_simulation)]
        group = [[np.random.randint(0, self.n),np.random.randint(0, self.n)]]
        for _ in range(int(self.n * self.n * self.density)):
            if np.random.uniform(0,1) < self.sparsity:
                obstacleFound = True
                neighbours = [[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]]
                while obstacleFound:
                    if len(neighbours) == 0:
                        group = [[np.random.randint(0, self.n), np.random.randint(0, self.n)]]
                        break
                    random_group = random.choice(group)
                    random_neighbour = random.choice(neighbours)
                    neighbours.remove(random_neighbour)

                    new_coord = list(np.array(random_neighbour) + np.array(random_group))
                    if 0 <= new_coord[0] < self.n and 0 <= new_coord[1] < self.n:
                        if len(self.grid[0][new_coord[0]][new_coord[1]]) == 0:
                            obstacleFound = False

                if not obstacleFound:
                    for t in range(self.t_simulation):
                        self.grid[t][new_coord[0]][new_coord[1]].append(True)
                        group.append(new_coord)

            else:
                obstacleFound = True
                while obstacleFound:

                    i = np.random.randint(0,self.n)
                    j = np.random.randint(0,self.n)

                    if not self.grid[0][i][j]:
                        obstacleFound = False

                for t in range(self.t_simulation):
                    self.grid[t][i][j].append(True)
                group = [[i,j]]