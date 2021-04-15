import time
import numpy as np
import gym
from tqdm import trange

import stanford_quad
from stanford_quad.common.ArmIK import ArmIK
from stanford_quad.common.Utilities import controller_to_sim
from stanford_quad.pupper.policy import HardPolicy
from stanford_quad.sim.simulator2 import PupperSim2
import matplotlib.pyplot as plt

# FREQ_CTRL = 60
# policy = HardPolicy(fps=FREQ_CTRL)
#
# iterations = 1000
# arm_pos = np.zeros((iterations, 3))
# arm_joints = np.zeros((iterations, 3))
#
# for i in trange(iterations):
#     _, action_arm = policy.act(velocity_horizontal=(0, 0), yaw_rate=0.0, normalized=True, velocity_arm=(0.0, -0.001, 0))
#     arm_pos[i] = np.copy(policy.state.arm_pos)
#     arm_joints[i] = np.copy(policy.state.arm_joints[:3])
#     # print(arm_joints[i])
#
# print(np.max(arm_joints))
# print(np.min(np.rad2deg(arm_joints)))
# print(np.max(np.rad2deg(arm_joints)))
#
#
# fig, axs = plt.subplots(2, 3)
# for i in range(3):
#     axs[0, i].plot(np.arange(iterations), arm_pos[:, i])
#     axs[0, i].set_title(f"{['x','y','z'][i]} pos")
#     axs[1, i].plot(np.arange(iterations), arm_joints[:, i])
#     axs[1, i].set_title(f"{i} joint")
#
# plt.show()

armik = ArmIK()

import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D


print(np.around(armik.chain.forward_kinematics((0, 0, 0, 0, 0))[:3, 3], 2))
# quit()

point = np.array((0, 0.16, 0.14))


for i in range(100):
    point[1] += 0.1
    ax = matplotlib.pyplot.figure().add_subplot(111, projection="3d")
    ax.set_xlim(-0.2, 0.2)
    ax.set_ylim(-0.2, 0.2)
    ax.set_zlim(0, 0.4)
    joints = armik.chain.inverse_kinematics(point)
    print(np.around(joints, 5))
    armik.chain.plot(joints, ax)
    matplotlib.pyplot.show()
