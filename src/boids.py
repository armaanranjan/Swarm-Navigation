import numpy as np

class Boid:
    def __init__(self, position, velocity, max_speed=3.0, perception_radius=60):
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.max_speed = max_speed
        self.perception_radius = perception_radius

    def limit_speed(self):
        speed = np.linalg.norm(self.velocity)
        if speed > self.max_speed:
            self.velocity = (self.velocity / speed) * self.max_speed

    def update(self, acceleration):
        self.velocity += acceleration
        self.limit_speed()
        self.position += self.velocity
