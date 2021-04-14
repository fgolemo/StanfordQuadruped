import time
import numpy as np
import gym
import stanford_quad
from stanford_quad.common.Utilities import controller_to_sim, ik_to_sim
from stanford_quad.pupper.policy import HardPolicy
from stanford_quad.sim.simulator2 import PupperSim2

FREQ_SIM = 240
FREQ_CTRL = 60
substeps = int(FREQ_SIM / FREQ_CTRL)
sim = PupperSim2(with_arm=True, frequency=FREQ_SIM, debug=True)
sim.reset()
# time.sleep(1)
policy = HardPolicy(fps=FREQ_CTRL, warmup=60)

arm_rest = np.array([0, 1, -0.8]) * np.pi / 2
sim.action_arm(arm_rest)

while True:
    # action, _ = controller_to_sim(policy.act(velocity_horizontal=(-0.2, 0), normalized=True)) * np.pi
    # action, _ = controller_to_sim(policy.act(velocity_horizontal=(0.2, 0), yaw_rate=1.0, normalized=True)) * np.pi
    action, action_arm = policy.act(
        velocity_horizontal=(0.2, 0), yaw_rate=1.0, normalized=True, velocity_arm=(+0.001, +0.002, 0)
    )
    # print(action_arm)
    action = controller_to_sim(action) * np.pi  # denormalize
    action_arm = ik_to_sim(action_arm)

    sim.action(action)
    sim.action_arm(action_arm)
    for _ in range(substeps):
        sim.step()
        # time.sleep(0.01)
