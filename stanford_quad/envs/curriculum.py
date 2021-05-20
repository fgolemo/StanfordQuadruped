from collections import deque

import gym
from gym import spaces
import numpy as np
import matplotlib.pyplot as plt
import random

from stanford_quad.envs.pupper_base import PupperBase
from stanford_quad.sim.procedural_generation.room import Room

class WalkRoomGoal(PupperBase):
    def __init__(self, **kwargs):
        super(WalkRoomGoal, self).__init__(**kwargs)
        self.room = None

    def _reset_simulation_env(self):
        if self.room:
            self.room.clear()

        pos = (1.5, 0, 0)
        size = (random.uniform(1, 2), random.uniform(1, 2), .1)
        color = (0.5, 1, 1)

        self.room = Room(self.sim.p, pos, size, color)
        self.room.add_inside_wall(False, 0.01, 0.4, (0, 0.5))
        self.room.add_inside_wall(False, 0.01, 0.4, (0.6, 1))
        self.room.generate_room()

