import time

import cv2
import numpy as np
import gym
from tqdm import trange

import stanford_quad
from stanford_quad.common.Utilities import controller_to_sim, ik_to_sim
from stanford_quad.pupper.policy import HardPolicy
from stanford_quad.sim.simulator2 import PupperSim2

FREQ_SIM = 240
FREQ_CTRL = 60
substeps = int(FREQ_SIM / FREQ_CTRL)
sim = PupperSim2(with_arm=True, frequency=FREQ_SIM, debug=False, img_size=(512, 256))
sim.reset()
sim.change_color((0.588, 0.419, 1, 1))
# time.sleep(1)
policy = HardPolicy(fps=FREQ_CTRL, warmup=10)

arm_rest = np.array([0, 1, -0.8]) * np.pi / 2
sim.action_arm(arm_rest)
arm_angles = []

for _ in range(5):
    sim.step()
    time.sleep(1)

for i in trange(60 * 47):

    # action, _ = controller_to_sim(policy.act(velocity_horizontal=(-0.2, 0), normalized=True)) * np.pi
    # action, _ = controller_to_sim(policy.act(velocity_horizontal=(0.2, 0), yaw_rate=1.0, normalized=True)) * np.pi
    idle_walk_fw = 0.008

    vel_body = (0, 0)
    yaw = 0
    vel_arm = (0, 0, 0)  # (+0.001, +0.002, 0)
    if 0 <= i <= 60 * 5:
        vel_body = (0.2, 0)
        txt = "forward"
    elif 60 * 5 < i <= 60 * 9:
        vel_body = (-0.2, 0)
        txt = "backward"
    elif 60 * 9 < i <= 60 * 14:
        vel_body = (0, -0.15)
        txt = "right"
    elif 60 * 14 < i <= 60 * 19:
        vel_body = (0.05, 0.15)
        txt = "left (and a bit fw)"
    elif 60 * 19 < i <= 60 * 24:
        yaw = -1
        txt = "turn right"
    elif 60 * 24 < i <= 60 * 29:
        yaw = 1
        txt = "turn left"
    elif 60 * 29 < i <= 60 * 31:
        vel_arm = (0, 0.001, 0)
        txt = "arm fw"
    elif 60 * 31 < i <= 60 * 34:
        vel_arm = (0, -0.001, 0)
        txt = "arm bw"
    elif 60 * 34 < i <= 60 * 36:
        vel_arm = (0.001, 0, 0)
        txt = "arm right"
    elif 60 * 36 < i <= 60 * 39:
        vel_arm = (-0.001, 0, 0)
        txt = "arm left"
    elif 60 * 39 < i <= 60 * 41:
        vel_arm = (0.001, 0, 0.001)
        txt = "arm up"
    elif 60 * 41 < i <= 60 * 47:
        vel_arm = (0, 0.0001, -0.001)
        txt = "arm down"
    else:
        quit()

    if 60 * 29 < i:
        vel_body = (idle_walk_fw, 0)

    action, action_arm = policy.act(velocity_horizontal=vel_body, yaw_rate=yaw, normalized=True, velocity_arm=vel_arm)
    # print(action_arm)
    action = controller_to_sim(action) * np.pi  # denormalize
    arm_angles.append(action_arm)
    # print(i // 60, policy.state.arm_pos, action_arm)
    sim.action(action)
    sim.action_arm(action_arm)
    for _ in range(substeps):
        sim.step()
        # time.sleep(0.01)

    if i % 2 == 0:
        img, _ = sim.take_photo(follow_bot=False, camera_offset=(0.5, 0.4, 0.4), lookat_offset=(0.5, 0, 0))
        font = cv2.FONT_HERSHEY_SIMPLEX
        img = np.array(img)
        # cv2.putText(img, txt, (200, 200), font, 1, (255, 106, 213), 2, cv2.LINE_AA)
        #cv2.putText(img, txt, (200, 200), font, 1, (0, 0, 0), 2, cv2.LINE_AA)
        # cv2.imshow("test", img[:, :, ::-1])
        # cv2.waitKey(1)
        cv2.imwrite(f"../imgs/experts-gif/{i:04d}.png", img[:, :, ::-1])
