import numpy as np

def obstacle_avoidance(boid, obstacles,
                       obs_weight=4.0,
                       influence_radius=80,
                       tangential_weight=0.5):

    force = np.zeros(2)

    for obs in obstacles:
        diff = boid.position - obs.position
        dist = np.linalg.norm(diff)
        if dist == 0:
            continue

        surface_dist = dist - obs.radius

        if surface_dist < influence_radius:
            normal = diff / dist
            repel = normal / max(surface_dist, 1e-2)**2
            tangent = np.array([-normal[1], normal[0]])
            force += repel + tangential_weight * tangent

    return obs_weight * force


def compute_forces(boid, boids, obstacles,
                   centralized=False,
                   noise_std=0.0,
                   sep_weight=1.2,
                   align_weight=0.8,
                   coh_weight=0.8,
                   obs_weight=4.0):

    separation = np.zeros(2)
    alignment = np.zeros(2)
    cohesion = np.zeros(2)

    if centralized:
        center = np.mean([b.position for b in boids], axis=0)
        avg_vel = np.mean([b.velocity for b in boids], axis=0)
        cohesion = center - boid.position
        alignment = avg_vel - boid.velocity

    else:
        neighbors = []
        for other in boids:
            if other is boid:
                continue

            noisy_pos = other.position + np.random.normal(0, noise_std, 2)
            dist = np.linalg.norm(boid.position - noisy_pos)

            if dist < boid.perception_radius and dist > 0:
                neighbors.append(other)
                separation += (boid.position - noisy_pos) / dist**2
                alignment += other.velocity
                cohesion += noisy_pos

        if neighbors:
            alignment /= len(neighbors)
            cohesion = (cohesion / len(neighbors)) - boid.position

    obs_force = obstacle_avoidance(boid, obstacles, obs_weight)

    return (
        sep_weight * separation +
        align_weight * alignment +
        coh_weight * cohesion +
        obs_force
    )
