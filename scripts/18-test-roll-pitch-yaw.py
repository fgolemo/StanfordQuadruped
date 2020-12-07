import numpy as np
from scipy.spatial.transform import Rotation

from stanford_quad.sim.simulator2 import PupperSim2, REST_POS

RECORDING_FPS = 60  # limited by camera
SIM_FPS = 240  # arbitrary but should be a multiple of the RECORDING_FPS

TRICK = "walk-forward"
FRAME_START = 90
FRAME_END = 400

RESOLUTION = 256

substeps = int(round(SIM_FPS / RECORDING_FPS))


sim = PupperSim2(debug=True, img_size=(RESOLUTION, RESOLUTION), frequency=SIM_FPS)
sim.reset(rest=True)
sim.make_kinematic()

roll = sim.p.addUserDebugParameter("roll", -np.pi, np.pi, 0)
pitch = sim.p.addUserDebugParameter("pitch", -np.pi, np.pi, 0)
yaw = sim.p.addUserDebugParameter("yaw", -np.pi, np.pi, 0)

txt = None

while True:

    if txt is not None:
        sim.p.removeUserDebugItem(txt)

    pos, rot, vel = sim.get_pos_orn_vel()
    rot = np.around(rot, 4)
    # print(sim.get_joint_states())

    roll_val = sim.p.readUserDebugParameter(roll)
    pitch_val = sim.p.readUserDebugParameter(pitch)
    yaw_val = sim.p.readUserDebugParameter(yaw)

    orn = Rotation.from_euler("XYZ", [roll_val, pitch_val, yaw_val])

    # sim_ref.set(joints_reference)
    sim.move_kinectic_body([0, 0, 0.183], orn.as_quat())
    sim.step()

    txt = sim.p.addUserDebugText(f"R {rot[0]}, P {rot[1]}, Y {rot[2]}", [0.5, 0, 0.25], [1, 0, 0], 3)
