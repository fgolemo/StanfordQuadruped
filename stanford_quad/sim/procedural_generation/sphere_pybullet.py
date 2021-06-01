import numpy as np

class SpherePybullet:

    def __init__(self, p, pos, radius, color=(1, 0, 0), collision=False):
        # we need to round or small float errors will explode the simulation
        self.p = p
        self.pos = np.around(pos, 4)
        self.radius = np.around(radius, 4)
        self.color = color
        self.collision=collision
        self.uid = None

    def create(self, static=True):
        """
        :return: pybullet uId of the object.
        """
        obj_visual = self.p.createVisualShape(
         shapeType=self.p.GEOM_SPHERE, rgbaColor=list(self.color) + [1], radius=self.radius
        )

        if self.collision:
            obj_collision = self.p.createCollisionShape(shapeType=self.p.GEOM_SPHERE, radius=self.radius)
        else:
            obj_collision = -1

        if static:
            mass = 0 # mass 0 = static in pybullet
        else:
            mass = 1

        obj = self.p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=obj_collision,
            baseVisualShapeIndex=obj_visual,
            basePosition=self.pos,
            useMaximalCoordinates=False
        )

        self.uid = obj

        return obj

    def clear(self):
        self.p.removeBody(self.uid)