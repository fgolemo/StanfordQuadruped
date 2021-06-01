import time

import gym
import stanford_quad  # need this unused import to get our custom environments
import numpy as np
from stanford_quad.sim.procedural_generation.room import Room

#env = gym.make("WalkRoomGoal-v0")
env = gym.make("WalkRoomGoalVisualLvl2-v0")


for i in range(2):
    state = env.reset()
    done = False
    while not done:
        action = env.action_space.sample()
        obs, rew, done, misc = env.step(action)
        print(
            f"action: {np.around(action,2)}, " f"obs: {np.around(obs,2)}, " f"rew: {np.around(rew,2)}, " f"done: {done}"
        )

env.close()

env = gym.make("RoomGoalReachLvl2-v0")


for i in range(2):
    state = env.reset()
    done = False
    while not done:
        action = env.action_space.sample()
        obs, rew, done, misc = env.step(action)
        print(
            f"action: {np.around(action,2)}, " f"obs: {np.around(obs,2)}, " f"rew: {np.around(rew,2)}, " f"done: {done}"
        )

env.close()

env = gym.make("WalkRoomGoalLvl2-v0")

for i in range(3):
    state = env.reset()
    done = False
    while not done:
        action = env.action_space.sample()
        obs, rew, done, misc = env.step(action)
        print(
            f"action: {np.around(action,2)}, " f"obs: {np.around(obs,2)}, " f"rew: {np.around(rew,2)}, " f"done: {done}"
        )
