import RRT_star as rrt
import helpers as hp

class Hub:
    def __init__(self):
        self.node = rrt.Node(hp.random_pos_collision_free(),None)
        self.graph = rrt.Graph(self.node, 0)