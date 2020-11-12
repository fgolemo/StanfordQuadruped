import time

import gym
import stanford_quad

env = gym.make("Pupper-Walk-Relative-aScale_1.0-freq_60-gFact_0.66-RandomZRot_10-Graphical-v0")

for i in range(3):
    env.reset()
    while True:
        _, _, done, _ = env.step([0] * 12)
        # time.sleep(0.1)
        if done:
            break
