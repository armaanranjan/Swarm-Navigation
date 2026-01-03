import numpy as np

def polarization(boids):
    directions = []
    for b in boids:
        speed = np.linalg.norm(b.velocity)
        if speed > 0:
            directions.append(b.velocity / speed)

    if not directions:
        return 0.0

    return np.linalg.norm(np.mean(directions, axis=0))

def collision_count(boids, obstacles, min_dist=8):
    agent_col = 0
    obs_col = 0

    for i, b1 in enumerate(boids):
        for b2 in boids[i+1:]:
            if np.linalg.norm(b1.position - b2.position) < min_dist:
                agent_col += 1

        for obs in obstacles:
            if np.linalg.norm(b1.position - obs.position) < obs.radius:
                obs_col += 1

    return agent_col, obs_col
