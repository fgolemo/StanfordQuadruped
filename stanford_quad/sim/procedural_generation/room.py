import numpy as np

from stanford_quad.sim.procedural_generation.floor import Floor
from stanford_quad.sim.procedural_generation.box_pybullet import BoxPybullet


class InsideWall:
    def __init__(self, x_y_axis, thickness, relative_pos, length_interval, height=None):
        """
        :param x_y_axis: boolean that indicate if the wall is in the x or y axis, True = x, False = y
        :param thickness: Thickness of the wall
        :param relative_pos: between 0 and 1, indicate the position of the wall inside the room
        :param length_interval: Tuple of 2 numbers between 0 and 1, first value should be small than second value,
                                , this tuple indicate the length of the wall relative to the room, example: (0, 0.5)
                                will be a wall with a length of half the room.
        """
        self.x_y_axis = x_y_axis
        self.thickness = thickness
        assert relative_pos >= 0 and relative_pos <= 1, "relative_pos should be between 0 and 1"
        self.relative_pos = relative_pos
        assert len(length_interval) == 2, "length interval should be a tuple of two."
        assert length_interval[0] >= 0 and length_interval[0] <= 1, "lenght_interval values should be between 0 and 1"
        assert length_interval[1] >= 0 and length_interval[1] <= 1, "lenght_interval values should be between 0 and 1"
        self.length_interval = length_interval
        self.height = height

class Room:

    def __init__(self, p, pos, floor_size, color, walls_height=0.3, walls_thickness=0.01, remove_walls=[]):
        self.p = p
        self.pos = pos
        self.orientation = [0, 0, 0]
        # half size
        self.floor_size = floor_size
        self.walls_height = walls_height
        self.walls_thickness = walls_thickness
        self.color = color
        self.inside_walls = []
        self.walls_uid = []
        self.remove_walls = remove_walls
        self.floor = Floor(self.p, self.pos, self.orientation, self.floor_size, self.color)

    def add_inside_wall(self, x_y_axis, thickness, relative_pos, length_interval, height=None):
        """
         Add an inside wall (see InsideWall parameters for parameters).
        """
        self.inside_walls.append(InsideWall(x_y_axis, thickness, relative_pos, length_interval, height=height))

    def generate_room(self):
        # Create room floor
        floor_uid = self.floor.create()
        self.generate_outside_walls()
        self.generate_inside_walls()


    def generate_outside_walls(self):
        # Create walls
        top_down_wall_size = (self.floor_size[0] + self.walls_thickness, self.walls_thickness, self.walls_height)

        top_down_wall = BoxPybullet(self.p, [0, 0, 0], self.orientation, top_down_wall_size, self.color, False)

        if not "top" in self.remove_walls:
            top_wall_uid = top_down_wall.create()
            self.walls_uid.append(top_wall_uid)
            self.floor.put_on(top_wall_uid, 0.5, 0, top_down_wall_size[2])

        if not "down" in self.remove_walls:
            down_wall_uid = top_down_wall.create()
            self.walls_uid.append(down_wall_uid)
            self.floor.put_on(down_wall_uid, 0.5, 1, top_down_wall_size[2])


        side_wall_size = ( self.walls_thickness, self.floor_size[1] - self.walls_thickness, self.walls_height)
        side_wall = BoxPybullet(self.p, [0, 0, 0], self.orientation, side_wall_size, self.color, False)

        if not "left" in self.remove_walls:
            left_wall_uid = side_wall.create()
            self.walls_uid.append(left_wall_uid)
            self.floor.put_on(left_wall_uid, 0, 0.5, side_wall_size[2])

        if not "right" in self.remove_walls:
            right_wall_uid = side_wall.create()
            self.walls_uid.append(right_wall_uid)
            self.floor.put_on(right_wall_uid, 1, 0.5, side_wall_size[2])

    def generate_inside_walls(self):
        for inside_wall in self.inside_walls:
            wall_length = inside_wall.length_interval[1] - inside_wall.length_interval[0]
            if inside_wall.height is not None:
                wall_height = inside_wall.height
            else:
                wall_height = self.walls_height
            if inside_wall.x_y_axis == True:
                wall_size = (self.floor_size[0] * wall_length, inside_wall.thickness,wall_height)
                wall_box = BoxPybullet(self.p, [0, 0, 0], self.orientation, wall_size, self.color, False)
                wall_uid = wall_box.create()
                self.walls_uid.append(wall_uid)
                self.floor.put_on(wall_uid,inside_wall.length_interval[0] + wall_length*0.5, inside_wall.relative_pos,
                                  wall_size[2])

            else:
                wall_size = (inside_wall.thickness, self.floor_size[1] * wall_length, wall_height)
                wall_box = BoxPybullet(self.p, [0, 0, 0], self.orientation, wall_size, self.color, False)
                wall_uid = wall_box.create()
                self.walls_uid.append(wall_uid)
                self.floor.put_on(wall_uid,inside_wall.relative_pos,
                                  inside_wall.length_interval[0] + wall_length*0.5, wall_size[2])


    def clear(self):
        self.floor.clear()
        for wall_uid in self.walls_uid:
            self.p.removeBody(wall_uid)