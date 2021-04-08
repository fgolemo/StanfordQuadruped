import math
import time

import numpy as np

from stanford_quad.sim.simulator2 import PupperSim2

FREQ_SIM = 240  # arbitrary but should be a multiple of the RECORDING_FPS
RESOLUTION = 256
FREQ_CTRL = 60

substeps = int(FREQ_SIM / FREQ_CTRL)

sim = PupperSim2(debug=True, img_size=(RESOLUTION, RESOLUTION), frequency=FREQ_SIM, with_arm=True)
sim.reset(rest=True)
sim.change_color((0, 1, 1, 1))
time.sleep(2)
for _ in range(10):
    sim.step()

motors = [24, 26, 28]

# Container for debug inputs
debugParams = []

## safe arm resting pos: [0, 1, -.85] * pi/2

# In the user interface, create a slider for each motor to control them separately.
for i in range(len(motors)):
    motor = sim.p.addUserDebugParameter("motor{}".format(i + 1), -1, 1, 0)
    debugParams.append(motor)

time.sleep(2)

for frame_idx in range(3000):

    # joints_learner = policy(last_observation) + joints_learner_base

    motorPos = []

    for i in range(len(motors)):
        pos = (math.pi / 2) * sim.p.readUserDebugParameter(debugParams[i])
        motorPos.append(pos)
        sim.p.setJointMotorControl2(sim.model, motors[i], sim.p.POSITION_CONTROL, targetPosition=pos)

    sim.action(sim.get_rest_pos())

    for _ in range(substeps):
        sim.step()

    time.sleep(0.1)

    # time.sleep

    # img_learner, segmap_learner = sim_learner.take_photo(with_segmap=True)
