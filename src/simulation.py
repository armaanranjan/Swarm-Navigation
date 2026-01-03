import pygame
import numpy as np

from boids import Boid
from obstacles import Obstacle
from swarm import compute_forces
from metrics import polarization, collision_count
from logger import DataLogger

# ===============================
# CONFIG
# ===============================
WIDTH, HEIGHT = 800, 600
FPS = 60
NUM_BOIDS = 40
NOISE_STD = 0.8          # ← SENSOR NOISE
CENTRALIZED = False     # ← TOGGLE THIS
STEPS = 0

# ===============================
# INIT
# ===============================
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

boids = [
    Boid(
        position=np.random.rand(2) * [WIDTH, HEIGHT],
        velocity=np.random.randn(2)
    )
    for _ in range(NUM_BOIDS)
]

obstacles = [
    Obstacle((400, 300), 60),
    Obstacle((200, 150), 40),
    Obstacle((600, 450), 50)
]

logger = DataLogger(
    filename=f"run_centralized_{CENTRALIZED}_noise_{NOISE_STD}.csv"
)

# ===============================
# LOOP
# ===============================
running = True
while running:
    clock.tick(FPS)
    screen.fill((30, 30, 30))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Draw obstacles
    for obs in obstacles:
        pygame.draw.circle(screen, (120, 120, 120),
                           obs.position.astype(int), obs.radius)

    # Update boids
    for boid in boids:
        acc = compute_forces(
            boid, boids, obstacles,
            centralized=CENTRALIZED,
            noise_std=NOISE_STD
        )
        boid.update(acc)
        boid.position[0] %= WIDTH
        boid.position[1] %= HEIGHT

        pygame.draw.circle(screen, (200, 200, 255),
                           boid.position.astype(int), 4)

    # Metrics
    pol = polarization(boids)
    agent_col, obs_col = collision_count(boids, obstacles)
    logger.log(STEPS, pol, agent_col, obs_col)
    STEPS += 1

    pygame.display.set_caption(
        f"Pol={pol:.2f} | AgentCol={agent_col} | ObsCol={obs_col}"
    )

    pygame.display.flip()

pygame.quit()
logger.close()
