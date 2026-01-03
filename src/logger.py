import csv
import os

class DataLogger:
    def __init__(self, filename):
        os.makedirs("results", exist_ok=True)
        self.file = open(f"results/{filename}", "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["step", "polarization", "agent_collisions", "obstacle_collisions"])

    def log(self, step, pol, agent_col, obs_col):
        self.writer.writerow([step, pol, agent_col, obs_col])

    def close(self):
        self.file.close()
