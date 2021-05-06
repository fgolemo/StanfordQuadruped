import numpy as np

from stanford_quad.sim.procedural_generation.floor import Floor
from stanford_quad.sim.procedural_generation.box_pybullet import BoxPybullet

class RoomGenerator:

    def __init__(self, p, pos, floor_size, color):
        self.p = p
        self.pos = pos
        self.orientation = [0, 0, 0]
        # half size
        self.floor_size = floor_size
        self.walls_height = 0.3
        self.walls_thickness = 0.01
        self.color = color

        self.floor = None

    def generate_room(self):

        # Create room floor
        self.floor = Floor(self.p, self.pos, self.orientation, self.floor_size, self.color)
        floor_uid = self.floor.create()
        self.generate_outside_walls()


    def generate_outside_walls(self):
        # Create walls
        top_down_wall_size = (self.floor_size[0], self.walls_thickness, self.walls_height)

        top_down_wall = BoxPybullet(self.p, [0, 0, 0], self.orientation, top_down_wall_size, self.color, False)
        top_wall_uid = top_down_wall.create()
        down_wall_uid = top_down_wall.create()

        self.floor.put_on(top_wall_uid, 0.5, 0, top_down_wall_size)
        self.floor.put_on(down_wall_uid, 0.5, 1, top_down_wall_size)

        side_wall_size = ( self.walls_thickness, self.floor_size[1], self.walls_height)
        side_wall = BoxPybullet(self.p, [0, 0, 0], self.orientation, side_wall_size, self.color, False)
        left_wall_uid = side_wall.create()
        right_wall_uid = side_wall.create()
        self.floor.put_on(left_wall_uid, 0, 0.5, side_wall_size)
        self.floor.put_on(right_wall_uid, 1, 0.5, side_wall_size)