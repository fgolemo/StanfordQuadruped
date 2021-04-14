import time

import numpy as np

from stanford_quad.common.Utilities import controller_to_sim
from stanford_quad.pupper.policy import HardPolicy

FREQ_CTRL = 60
TEST_BODY = False
TEST_ARM = True

policy = HardPolicy(fps=FREQ_CTRL)


########### BODY
if TEST_BODY:
    iterations = 10_000
    times = 0
    for _ in range(iterations):
        start = time.time()
        _ = controller_to_sim(policy.act(velocity_horizontal=(0.2, 0), yaw_rate=1.0, normalized=True)) * np.pi
        diff = time.time() - start
        times += diff

    print(
        f"BODY: ran {iterations} iteractions at {np.around(iterations/times,1)}Hz, "
        f"so avg of {times/iterations}s per iterations"
    )
    # MBP 2018, ran 100000 iteractions at 1749.8Hz, so avg of 0.000571490352153778s per iterations

########### ARM
if TEST_ARM:
    iterations = 100
    times = 0
    for _ in range(iterations):
        start = time.time()
        _ = controller_to_sim(policy.act(velocity_arm=(0.0, 0.001, 0), normalized=True)) * np.pi
        diff = time.time() - start
        times += diff

    print(
        f"ARM: ran {iterations} iteractions at {np.around(iterations/times,1)}Hz, "
        f"so avg of {times/iterations}s per iterations"
    )
