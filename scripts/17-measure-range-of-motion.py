import cmath

from stanford_quad.common.Utilities import controller_to_sim
from stanford_quad.pupper.policy import HardPolicy
import numpy as np
import matplotlib.pyplot as plt

policy = HardPolicy()
steps = 200
start = 50

joints = []
feet = []
for _ in range(steps):
    action = controller_to_sim(policy.act(velocity_horizontal=(0.6, 0), normalized=True))
    joints.append(action)
    feet.append(policy.state.foot_locations.T)

actions = np.array(joints)
feets = np.array(feet)

x = np.arange(start, steps)
joint_ranges = []
foot_ranges = []
for i in range(12):
    # plt.plot(x, actions[:, i], label=f"joint {i+1}")
    joint_ranges.append((min(actions[start:, i]), max(actions[start:, i])))

for i in range(4):
    x_range = (min(feets[start:, i, 0]), max(feets[start:, i, 0]))
    y_range = (min(feets[start:, i, 1]), max(feets[start:, i, 1]))  # actually z
    z_range = (min(feets[start:, i, 2]), max(feets[start:, i, 2]))  # actually y
    foot_ranges.append((x_range, y_range, z_range))
    print(foot_ranges[-1])


np.savez("../stanford_quad/assets/joint_foot_ranges.npz", joint_ranges=joint_ranges, foot_ranges=foot_ranges)

plt.scatter(feets[:, 0, 0], feets[:, 0, 2])
plt.show()


# plt.legend()
# plt.tight_layout()
# plt.show()

# for r in ranges:
#     print(r)

# joint = 2
# freq: 0.0
# freq: 0.20943951023931953
# freq: 0.41887902047863906
# freq: 0.6283185307179586
# freq: 0.8377580409572781

joint = 1
# freq: 0.0
# freq: 0.20943951023931953
# freq: 0.41887902047863906
# freq: 0.6283185307179586

# plt.plot(x, actions[start:, joint], label="original")

# data = actions[start:, joint]
# fft3 = np.fft.fft(data)
# freqs = np.fft.fftfreq(len(data))
# threshold = 0.01
# recomb = np.zeros((len(data),))
# middle = len(x) // 2 + 1
# for i in range(middle):
#     if abs(fft3[i]) / (len(x)) > threshold:
#         if i == 0:
#             coeff = 2
#         else:
#             coeff = 1
#         print("freq:", freqs[i] * 2 * np.pi)
#         sinusoid = 1 / (len(x) * coeff / 2) * (abs(fft3[i]) * np.cos(freqs[i] * 2 * np.pi * x + cmath.phase(fft3[i])))
#         recomb += sinusoid
#         plt.plot(x, sinusoid)
#
# plt.plot(x, recomb, label="recombination")
# plt.legend()
# plt.show()
