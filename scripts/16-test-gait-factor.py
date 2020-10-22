import time

import gym
import stanford_quad

env = gym.make("Pupper-Walk-Relative-aScale_1.0-gFact_0.2-RandomZRot_0-Graphical-v0")

for i in range(3):
    env.reset()
    while True:
        _, _, done, _ = env.step([0] * 12)
        # time.sleep(0.1)
        if done:
            break
