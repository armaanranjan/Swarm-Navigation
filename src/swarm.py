import numpy as np


def compute_forces(boid, boids, obstacles,
                   sep_weight=1.5,
                   align_weight=1.0,
                   coh_weight=1.0,
                   obs_weight=2.0):

    separation = np.zeros(2)
    alignment = np.zeros(2)
    cohesion = np.zeros(2)

    neighbors = []

    for other in boids:
        if other is boid:
            continue
        distance = np.linalg.norm(boid.position - other.position)
        if distance < boid.perception_radius and distance > 0:
            neighbors.append(other)
            separation += (boid.position - other.position) / (distance ** 2)
            alignment += other.velocity
            cohesion += other.position

    if neighbors:
        alignment /= len(neighbors)
        cohesion = (cohesion / len(neighbors)) - boid.position

    obs_force = obstacle_avoidance(boid, obstacles, obs_weight)

    acceleration = (
        sep_weight * separation +
        align_weight * alignment +
        coh_weight * cohesion +
        obs_force
    )

    return acceleration

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
            # Normalized outward normal
            normal = diff / dist

            # Strong nonlinear repulsion
            repel = normal / max(surface_dist, 1e-2)**2

            # Tangential force (perpendicular)
            tangent = np.array([-normal[1], normal[0]])

            force += repel + tangential_weight * tangent

    return obs_weight * force


