import numpy as np

from stanford_quad.sim.procedural_generation.floor import Floor

class RoomGenerator:

    def __init__(self, p, pos, orientation, size, color):
        self.p = p
        self.pos = pos
        self.orientation = orientation
        self.size = size
        self.color = color

        self.floor = None

    def generate_room(self):

        self.floor = Floor(self.p, self.pos, self.orientation, self.size, self.color)
        floor_uid = self.floor.create()


        """
        floor_obj = BoxPybullet(self.p, self.pos, self.orientation, self.size, self.color, False).create()

        # and fix the first step to the ground/world
        self.p.createConstraint(
            parentBodyUniqueId=floor_obj,
            parentLinkIndex=-1,
            childBodyUniqueId=-1,
            childLinkIndex=-1,
            jointType=self.p.JOINT_FIXED,
            jointAxis=(1, 1, 1),
            parentFramePosition=-np.array([0, 0, size[2]]),#np.array(pos),
            childFramePosition=pos,
        ) """
