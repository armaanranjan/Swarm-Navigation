import numpy as np

def polarization(boids):
    directions = []

    for boid in boids:
        speed = np.linalg.norm(boid.velocity)
        if speed > 0:
            directions.append(boid.velocity / speed)

    if not directions:
        return 0.0

    mean_direction = np.mean(directions, axis=0)
    return np.linalg.norm(mean_direction)

def collision_count(boids, obstacles, min_dist=8):
    agent_collisions = 0
    obstacle_collisions = 0

    for i, b1 in enumerate(boids):
        for b2 in boids[i+1:]:
            if np.linalg.norm(b1.position - b2.position) < min_dist:
                agent_collisions += 1

        for obs in obstacles:
            if np.linalg.norm(b1.position - obs.position) < obs.radius:
                obstacle_collisions += 1

    return agent_collisions, obstacle_collisions
