from collections import deque

import gym
from gym import spaces
import numpy as np
import matplotlib.pyplot as plt
import random
import os

from stanford_quad.envs.pupper_base import PupperBase
from stanford_quad.sim.procedural_generation.room import Room
from stanford_quad.sim.procedural_generation.sphere_pybullet import SpherePybullet

from stanford_quad.assets import ASSET_DIR

class WalkRoomGoal(PupperBase):
    def __init__(self, inside_walls=False, **kwargs):
        super(WalkRoomGoal, self).__init__(**kwargs)
        self.room = None
        self.target = None
        self.target_pos = None
        self.inside_walls = inside_walls

    def _reset_simulation_env(self):

        if self.room:
            self.room.clear()

        if self.target:
            self.target.clear()

        pos = (1.5, 0, 0)
        size = (random.uniform(1, 2), random.uniform(1, 2), .1)
        color = (0.8, 0.8, 0.8)

        self.room = Room(self.sim.p, pos, size, color)

        if self.inside_walls:
            if random.random() < 0.5:
                lr_side = True
            else:
                lr_side = False

            wall_opening_position = random.uniform(0.2, 1.0)
            random_inside_wall_position = random.uniform(0.1, 0.9)
            self.room.add_inside_wall(lr_side, 0.01, random_inside_wall_position, (0, random_inside_wall_position - 0.2))
            self.room.add_inside_wall(lr_side, 0.01, random_inside_wall_position, (random_inside_wall_position, 1))

        self.room.generate_room()

        # Random pupper init pos
        self.room.floor.put_on(self.sim.model,random.uniform(0.1, 0.9), random.uniform(0.1, 0.9), 0.2)

        self.target = SpherePybullet(self.sim.p, [0, 0, 0], 0.05)
        self.target.create()

        random_x_pos = random.uniform(0.1, 0.9)
        random_y_pos = random.uniform(0.1, 0.9)

        self.target_pos = self.room.floor.get_world_pos(random_x_pos, random_y_pos)
        self.room.floor.put_on(self.target.uid, random_x_pos, random_y_pos, 0.1)

class RoomGoalReach(PupperBase):
    def __init__(self, inside_walls=False, **kwargs):
        super(RoomGoalReach, self).__init__(**kwargs)
        self.room = None
        self.target = None
        self.target_pos = None
        self.target_pos_var = 0.2
        self.inside_walls = inside_walls

    def _reset_simulation_env(self):

        if self.room:
            self.room.clear()

        if self.target:
            self.target.clear()

        pos = (1.5, 0, 0)
        size = (1, 1, .1)
        color = (0.8, 0.8, 0.8)

        self.room = Room(self.sim.p, pos, size, color)

        if self.inside_walls:
            if random.random() < 0.5:
                lr_side = True
            else:
                lr_side = False

            wall_opening_position = random.uniform(0.2, 1.0)
            random_inside_wall_position = random.uniform(0.1, 0.9)
            self.room.add_inside_wall(lr_side, 0.01, random_inside_wall_position,
                                      (0, wall_opening_position - 0.2))
            self.room.add_inside_wall(lr_side, 0.01, random_inside_wall_position, (wall_opening_position, 1))

        self.room.generate_room()

        # Random pupper init pos
        self.room.floor.put_on(self.sim.model, 0.5, 0.5, 0.2)

        random_x_pos = random.uniform(0.1, 0.9)
        random_y_pos = random.uniform(0.1, 0.9)

        middle_room_pos = self.room.floor.get_world_pos(random_x_pos, random_y_pos)
        middle_room_pos[2] = middle_room_pos[2] + 0.1
        self.target_pos = [middle_room_pos[0] + random.uniform(-self.target_pos_var,self.target_pos_var),
                           middle_room_pos[1] + random.uniform(-self.target_pos_var,self.target_pos_var),
                           middle_room_pos[2] + random.uniform(0,self.target_pos_var * 2)]

        self.target = SpherePybullet(self.sim.p, self.target_pos, 0.01, color=(0, 0, 1))
        self.target.create()


class WalkRoomGoalVisual(PupperBase):
    def __init__(self, inside_walls=False, **kwargs):
        super(WalkRoomGoalVisual, self).__init__(**kwargs)
        self.room = None
        self.target = None
        self.target_pos = None
        self.inside_walls = inside_walls

    def _reset_simulation_env(self):

        if self.room:
            self.room.clear()

        if self.target:
            self.sim.p.removeBody(self.target)

        pos = (1.5, 0, 0)
        size = (random.uniform(1, 2), random.uniform(1, 2), .1)
        color = (0.8, 0.8, 0.8)

        self.room = Room(self.sim.p, pos, size, color)

        if self.inside_walls:
            if random.random() < 0.5:
                lr_side = True
            else:
                lr_side = False

            wall_opening_position = random.uniform(0.2, 1.0)
            random_inside_wall_position = random.uniform(0.1, 0.9)
            self.room.add_inside_wall(lr_side, 0.01, random_inside_wall_position, (0, wall_opening_position - 0.2))
            self.room.add_inside_wall(lr_side, 0.01, random_inside_wall_position, (wall_opening_position, 1))

        self.room.generate_room()

        # Random pupper init pos
        self.room.floor.put_on(self.sim.model,random.uniform(0.1, 0.9), random.uniform(0.1, 0.9), 0.2)

        self.target = self.sim.p.loadURDF(os.path.join(ASSET_DIR, "/table/table.urdf"), basePosition=[0, 0, 0],
                                    globalScaling=0.3)

        random_x_pos = random.uniform(0.1, 0.9)
        random_y_pos = random.uniform(0.1, 0.9)

        self.target_pos = self.room.floor.get_world_pos(random_x_pos, random_y_pos)
        self.room.floor.put_on(self.target, random_x_pos, random_y_pos, 0.0)


class TwoRooms(PupperBase):
    def __init__(self, inside_walls=False, **kwargs):
        super(TwoRooms, self).__init__(**kwargs)
        self.room = None
        self.room_2 = None
        self.target = None
        self.target_pos = None
        self.inside_walls = inside_walls

    def _reset_simulation_env(self):

        if self.room:
            self.room.clear()

        if self.room_2:
            self.room_2.clear()

        if self.target:
            self.sim.p.removeBody(self.target)

        pos = (1.5, 0, 0)
        size = (random.uniform(1, 2), random.uniform(1, 2), .1)
        color = (0.8, 0.8, 0.8)

        self.room = Room(self.sim.p, pos, size, color, remove_walls=["down"])

        connecting_point_room_2 = self.room.floor.get_world_pos(0.5, 1)

        self.room.add_inside_wall(True, 0.2, 1, (0, 1), height=0.075)
        self.room.add_inside_wall(True, 0.2, 0.9, (0, 1), height=0.05)
        self.room.add_inside_wall(True, 0.2, 0.8, (0, 1), height=0.025)

        self.room.generate_room()

        room_2_size = (size[0], random.uniform(1, 2), .2)

        room_2_pos = connecting_point_room_2
        room_2_pos[1] += room_2_size[1]

        self.room_2 = Room(self.sim.p, room_2_pos, room_2_size, color=(0.5, 0.5, 0.5), remove_walls=["top"])
        self.room_2.generate_room()

        # Random pupper init pos
        self.room.floor.put_on(self.sim.model,random.uniform(0.1, 0.9), random.uniform(0.1, 0.9), 0.2)

        self.target = self.sim.p.loadURDF(os.path.join(ASSET_DIR, "/table/table.urdf"), basePosition=[0, 0, 0],
                                    globalScaling=0.3)

        random_x_pos = random.uniform(0.1, 0.9)
        random_y_pos = random.uniform(0.1, 0.9)

        self.target_pos = self.room_2.floor.get_world_pos(random_x_pos, random_y_pos)
        self.room_2.floor.put_on(self.target, random_x_pos, random_y_pos, 0.1)