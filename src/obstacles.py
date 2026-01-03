import numpy as np

class Obstacle:
    def __init__(self, position, radius):
        self.position = np.array(position, dtype=float)
        self.radius = radius
