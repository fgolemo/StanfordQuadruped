import numpy as np

from stanford_quad.sim.procedural_generation.box_pybullet import BoxPybullet

class Floor:

    def __init__(self, p, pos, orientation, size, color):
        """
         Floor create BoxPybullet attached to the ground, it can attach/spawn objects on it's surface.
        :param p:
        :param pos:
        :param orientation:
        :param size:
        :param color:
        """
        self.p = p
        self.pos = pos
        self.orientation = orientation
        self.size = size
        self.color = color
        self.uid = None

    def create(self):
        floor_obj = BoxPybullet(self.p, self.pos, self.orientation, self.size, self.color, False).create()

        # Fix the floor to the ground/world
        self.p.createConstraint(
            parentBodyUniqueId=floor_obj,
            parentLinkIndex=-1,
            childBodyUniqueId=-1,
            childLinkIndex=-1,
            jointType=self.p.JOINT_FIXED,
            jointAxis=(1, 1, 1),
            parentFramePosition=-np.array([0, 0, self.size[2]]),  # np.array(pos),
            childFramePosition=pos,
        )

        self.uid = floor_obj

        return floor_obj