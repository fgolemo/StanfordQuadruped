import numpy as np

class BoxPybullet:

    def __init__(self, p, pos, orn, size, color=(1, 0, 0), random_color=False, visible=True):
        # we need to round or small float errors will explode the simulation
        self.p = p
        self.pos = np.around(pos, 4)
        self.size = np.around(size, 4)
        self.orn = np.around(self.p.getQuaternionFromEuler(orn), 4)
        self.color = color

        if random_color:
         self.color = random_bright_color(uint=False)

        self.visible = visible

        self.uid = None

    def create(self, static=True):
        """
        :return: pybullet uId of the object.
        """
        obj_visual = -1
        if self.visible:
            obj_visual = self.p.createVisualShape(
             shapeType=self.p.GEOM_BOX, rgbaColor=list(self.color) + [1], halfExtents=self.size
            )

        obj_collision = self.p.createCollisionShape(shapeType=self.p.GEOM_BOX, halfExtents=self.size)

        if static:
            mass = 0 # mass 0 = static in pybullet
        else:
            mass = 1

        obj = self.p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=obj_collision,
            baseVisualShapeIndex=obj_visual,
            basePosition=self.pos,
            baseOrientation=self.orn,
            useMaximalCoordinates=False
        )

        self.uid = obj

        return obj