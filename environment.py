

class Environment:
    def __init__(self):
        self.drones = []
        self.hubs = []
        self.grid = None
        self.t_simulation = 1000
        self.t_expand = 2000
        self.n_hubs = 5

env = Environment()
