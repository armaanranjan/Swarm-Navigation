import pygame
import numpy as np

from boids import Boid
from swarm import compute_forces
from obstacles import Obstacle
from metrics import polarization, collision_count

# ===============================
# PYGAME SETUP
# ===============================
WIDTH, HEIGHT = 800, 600
FPS = 60

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Boids Swarm Simulation")
clock = pygame.time.Clock()

# ===============================
# SWARM PARAMETERS
# ===============================
NUM_BOIDS = 30

# ===============================
# CREATE BOIDS
# ===============================
boids = []
for _ in range(NUM_BOIDS):
    position = np.random.rand(2) * np.array([WIDTH, HEIGHT])
    velocity = np.random.randn(2)
    boids.append(Boid(position, velocity))

# ===============================
# CREATE OBSTACLES
# ===============================
obstacles = [
    Obstacle((400, 300), 50),
    Obstacle((200, 150), 40),
    Obstacle((600, 450), 60)
]

# ===============================
# MAIN LOOP
# ===============================
running = True
while running:
    clock.tick(FPS)
    screen.fill((30, 30, 30))

    # ---- Handle Events ----
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # ---- Draw Obstacles ----
    for obs in obstacles:
        pygame.draw.circle(
            screen,
            (100, 100, 100),
            obs.position.astype(int),
            obs.radius
        )

    # ---- Update Boids ----
    for boid in boids:
        acceleration = compute_forces(
        boid,
        boids,
        obstacles,
        sep_weight=1.2,
        align_weight=0.8,
        coh_weight=0.8,
        obs_weight=4.0
    )


        boid.update(acceleration)

        # Wrap-around boundary conditions
        boid.position[0] %= WIDTH
        boid.position[1] %= HEIGHT

        # ---- Draw Boid ----
        pygame.draw.circle(
            screen,
            (200, 200, 255),
            boid.position.astype(int),
            4
        )

        # ---- Draw Heading Vector ----
        vel_norm = np.linalg.norm(boid.velocity)
        if vel_norm > 0:
            heading = boid.velocity / vel_norm
            end_pos = boid.position + heading * 12

            pygame.draw.line(
                screen,
                (255, 100, 100),
                boid.position.astype(int),
                end_pos.astype(int),
                2
            )

    # ===============================
    # METRICS
    # ===============================
    pol = polarization(boids)
    agent_col, obs_col = collision_count(boids, obstacles)

    pygame.display.set_caption(
        f"Boids Swarm | Pol: {pol:.2f} | Agent Col: {agent_col} | Obs Col: {obs_col}"
    )

    pygame.display.flip()

pygame.quit()
