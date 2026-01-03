# Boids-Based Swarm Navigation with Obstacle Avoidance

## Problem Statement

Swarm robotics studies the collective behavior of **multiple autonomous agents** that interact locally to achieve **emergent global behavior**. A fundamental challenge in swarm robotics is designing **decentralized control rules** that allow agents to:

1. Flock cohesively without collisions
2. Avoid obstacles in complex environments
3. Maintain robustness under uncertainty

The goal of this project is to simulate a **Boids-inspired swarm system** that demonstrates **emergent flocking behavior** and safe navigation using only local information, **without any central controller**.

---

## Solution Overview

This project implements a **decentralized Boids swarm simulation** with **obstacle avoidance**, combining:

- **Separation**: Each agent avoids crowding neighbors
- **Alignment**: Each agent matches its velocity with nearby agents
- **Cohesion**: Each agent moves toward the center of nearby agents
- **Obstacle Avoidance**: Agents repel from nearby obstacles with tangential escape vectors to navigate around them smoothly

The agents follow **simple local interaction rules**, yet the swarm exhibits **emergent behavior** such as flocking, coordinated movement, and obstacle avoidance.

The system is designed to be:

- **Fully decentralized** — no agent has global knowledge
- **Dynamic and robust** — capable of handling multiple agents and obstacles
- **Quantifiable** — metrics such as polarization, agent-agent collisions, and agent-obstacle collisions are computed in real time

---

## Implementation Details

### Architecture

boids-swarm-robotics/
│
├── src/
│ ├── boid.py # Defines Boid agent properties and update logic
│ ├── swarm.py # Computes local forces: separation, alignment, cohesion, obstacle avoidance
│ ├── obstacles.py # Defines obstacle positions and radii
│ ├── metrics.py # Computes polarization and collision metrics
│ └── simulation.py # Main simulation loop with Pygame visualization
├── experiments/ # Experiment notes and parameter tests
├── results/ # Screenshots, videos, and plots
├── requirements.txt # Python dependencies
└── README.md # Project documentation

yaml
Copy code

### Algorithm

Each agent computes its acceleration based on:

1. **Separation**  
   Prevents collisions with neighbors:
   \[
   F_{sep} = \sum_{j \in N_i} \frac{x_i - x_j}{\|x_i - x_j\|^2}
   \]

2. **Alignment**  
   Aligns velocity with neighbors:
   \[
   F_{align} = \bar{v}_{N_i} - v_i
   \]

3. **Cohesion**  
   Moves toward the local center of mass:
   \[
   F_{coh} = \bar{x}_{N_i} - x_i
   \]

4. **Obstacle Avoidance**  
   Repels from obstacles while allowing tangential sliding:
   \[
   F_{obs} = \sum_k \frac{x_i - o_k}{d^2} + \alpha \hat{t}
   \]

The **net acceleration** is a weighted sum of all forces:

\[
a_i = w_s F_{sep} + w_a F_{align} + w_c F_{coh} + w_o F_{obs}
\]

---

## Metrics

To quantify swarm behavior:

1. **Polarization**: Measures alignment of the swarm  
   \[
   P = \frac{1}{N} \left\| \sum_{i=1}^{N} \frac{v_i}{\|v_i\|} \right\|
   \]  
   P ≈ 1 → perfectly aligned; P ≈ 0 → random motion

2. **Agent-Agent Collisions**: Number of agents closer than a minimum safe distance

3. **Agent-Obstacle Collisions**: Number of agents colliding with obstacles

Metrics are displayed **in real-time** in the Pygame window.

---

## Visualization

- **Blue dots**: Boids  
- **Red lines**: Velocity/heading vectors  
- **Grey circles**: Obstacles  
- Real-time metrics appear in the window title

---

## How to Run

1. Clone the repository:

```bash
git clone https://github.com/<your-username>/Swarm-Navigation.git
cd Swarm-Navigation
Install dependencies:

bash
Copy code
pip install -r requirements.txt
Run the simulation:

bash
Copy code
python src/simulation.py
Experiments and Extensions
Test flocking behavior with/without alignment, cohesion, or separation

Test obstacle avoidance under different obstacle densities

Vary obs_weight, sep_weight, coh_weight to study robustness

## Future Work:
- Add sensor noise
- Log metrics to CSV for plots
- Compare centralized vs decentralized control
- Multi-agent reinforcement learning

## References
- Reynolds, C. W. (1987). Flocks, Herds, and Schools: A Distributed Behavioral Model. ACM SIGGRAPH.

- Brambilla, M., Ferrante, E., Birattari, M., & Dorigo, M. (2013). Swarm robotics: a review from the swarm engineering perspective. Swarm Intelligence, 7(1), 1–41.

- Couzin, I. D., Krause, J., Franks, N. R., & Levin, S. A. (2005). Effective leadership and decision-making in animal groups on the move. Nature, 433, 513–516.

- Trianni, V. (2014). Evolutionary Swarm Robotics: Evolving Self-Organizing Behaviors in Groups of Autonomous Robots. Springer.

## Author
- Armaan Ranjan
- B.Tech CSE (Data Science), SRM Institute of Science and Technology
- GitHub: https://github.com/armaanranjan

