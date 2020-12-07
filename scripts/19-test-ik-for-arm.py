import ikpy
import numpy as np
import ikpy.utils.plot as plot_utils
import time
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np


class ArmIK:
    def __init__(self):
        self.chain = Chain(name='pupper_ergo', links=[
            OriginLink(),
            URDFLink(
                name="twistybase",
                translation_vector=[0, 0, .03],
                orientation=[0, 0, 0],
                rotation=[0, 0, 1],
            ),
            URDFLink(
                name="shoulder",
                translation_vector=[0, 0, .0285],
                orientation=[0, 0, 0],
                rotation=[1, 0, 0],
            ),
            URDFLink(
                name="elbow",
                translation_vector=[0, 0, .085],
                orientation=[0, 0, 0],
                rotation=[1, 0, 0],
            ),
            URDFLink(
                name="tip",
                translation_vector=[0, .155, 0],
                orientation=[1.57, 0, 0],
                rotation=[1, 0, 0],
            )
        ])

    @staticmethod
    def _ik2pupperarm(joints):
        return np.rad2deg([joints[1], -joints[2], -joints[3], 0.0])

    @staticmethod
    def _makeMoveCmd(degs):
        print(f"move([{','.join([str(x) for x in np.around(degs, 2)])}])")

    def pos2joints(self, target):
        joints = self.chain.inverse_kinematics(target)
        return self._ik2pupperarm(joints)


armik = ArmIK()

# target_position = np.array([.05, .12, .14])
target_position = np.array([-.1, .17, .18])

iterations = 1000
start = time.time()
for _ in range(iterations):
    joints = armik.pos2joints(target_position)
diff = time.time() - start
print (f"that took {diff/iterations}s on avg, so {1/(diff/iterations)}Hz")

print (joints)
armik._makeMoveCmd(joints)

import matplotlib.pyplot as plt

fig, ax = plot_utils.init_3d_figure()
my_chain.plot(my_chain.inverse_kinematics(target_position), ax, target=target_position)
ax.set_xlim(-.2, .2)
ax.set_ylim(-.2, .2)
# plt.ylim(0, 150)
# plt.zlim(0, 150)
ax.set_zlim(-0,.4)
plt.show()
