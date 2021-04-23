import gym
import h5py
import numpy as np
from scipy.spatial.transform import Rotation
import cv2

from stanford_quad.envs.utils import rendermode_from_string, RenderMode
from stanford_quad.sim.HardwareInterface import HardwareInterface
from stanford_quad.sim.simulator2 import PupperSim2, FREQ_SIM
from stanford_quad.envs.imitation_recordings import IMITATION_LIB

CONTROL_FREQUENCY = 60
# RECORDINGS_PATH = "/Users/florian/dev/pyvicon/scripts/pupper-{}.hdf5"
RECORDINGS_PATH = "/home/gberseth/playground/StanfordQuadruped/pupper-{}.hdf5"
RESOLUTION = 48
SIM_AGENT_COLOR = (0, 1, 1, 1)
SIM_REF_COLOR = (1, 0, 1, 1)
JOINT_ERROR_SCALE = 5.0  # copied over from https://github.com/google-research/motion_imitation/blob/master/motion_imitation/envs/env_wrappers/imitation_task.py#L59
CAMERA_OFFSET = (-0.03, -0.275, -0.05)
LOOKAT_OFFSET = (-0.03, 0, -0.05)
MASKED = True  # do you want to cut out the robot from the background


def get_recording_joints(joints, frame_idx):
    joint_angles = np.reshape(joints[frame_idx], (3, 4))
    joint_angles_robot = HardwareInterface.parallel_to_serial_joint_angles(joint_angles)

    return joint_angles_robot.T.flatten()


def get_recording_pose(vicon_positions, base_pos, frame_idx):
    diff_pos = vicon_positions[frame_idx] - base_pos
    diff_pos /= 500  # for some weird reason it's not /100
    z = diff_pos[2] + 0.21  # (compensating for the model)
    x = diff_pos[1]
    y = diff_pos[0]

    # fixed manual rotation for now
    rot = Rotation.from_quat([0, 0, 0, 1])
    return (x, y, z), rot


def merge_images(img_ref, img_agent):
    # the "+2" is only there to create some space between the frames
    img_merged = np.zeros((RESOLUTION, RESOLUTION * 2 + 2, 3), np.uint8)
    img_merged[:, :RESOLUTION, :] = img_ref[:, :, ::-1]
    img_merged[:, RESOLUTION + 2 :, :] = img_agent[:, :, ::-1]
    return img_merged


class ImitationEnv(gym.Env):
    def __init__(self, trick, control_freq=CONTROL_FREQUENCY, action_scaling=0.5):
        super().__init__()

        self.control_freq = control_freq
        self.action_scaling = action_scaling

        f = h5py.File(RECORDINGS_PATH.format(trick), "r")

        self.vicon_positions = f["vicon_positions"]
        self.vicon_rotations = f["vicon_rotations"]

        self.idx_start = IMITATION_LIB[trick]["start"]
        self.idx_end = IMITATION_LIB[trick]["end"]

        self.base_pos = self.vicon_positions[self.idx_start]
        self.base_rot = self.vicon_rotations[self.idx_end]

        self.joints = f["joints"]
        self.feet = f["foot_positions"]

        # observation space:
        # - 12 lef joints in the order
        #   - front right hip
        #   - front right upper leg
        #   - front right lower leg
        #   - front left hip/upper/lower leg
        #   - back right hip/upper/lower leg
        #   - back left hip/upper/lower leg
        # - 3 body orientation in euler angles
        # - 2 linear velocity (only along the plane, we don't care about z velocity

        self.observation_space = gym.spaces.Box(low=-1, high=1, shape=(12 + 3 + 2,), dtype=np.float32)

        # action = 12 joint angles in the same order as above (fr/fl/br/bl and each with hip/upper/lower)
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(12,), dtype=np.float32)

        self.sim_agent = PupperSim2(
            debug=False,
            start_standing=False,
            gain_pos=1 / 16,
            gain_vel=1 / 8,
            max_torque=1 / 2,
            img_size=(RESOLUTION, RESOLUTION),
        )
        self.sim_agent.change_color((SIM_AGENT_COLOR))
        self.dt = 1 / self.control_freq
        self.sim_steps = int(round(FREQ_SIM / control_freq))

        self.sim_ref = PupperSim2(debug=False, img_size=(RESOLUTION, RESOLUTION))
        self.sim_ref.reset()
        # make the robot limp for demonstration
        self.sim_ref.make_kinematic(SIM_REF_COLOR)

        self.episode_steps = 0
        self.frame_idx = self.idx_start

    def _get_obs(self):
        pos, orn, vel = self.sim_agent.get_pos_orn_vel()

        # to normalize to [-1,1]
        joint_states = np.array(self.sim_agent.get_joint_states()) / np.pi
        obs = list(joint_states) + list(orn) + list(vel)[:2]
#         img = self.getVisualState()
        return np.array(obs)

    def reset(self):
        self.frame_idx = self.idx_start

        # reset the learning agent
        self.sim_agent.reset(rest=True)

        for _ in range(10):
            self.sim_agent.step()

        return self._get_obs()

    def _sanitize_actions(self, actions):
        assert len(actions) == 12
        scaled = actions * np.pi * self.action_scaling  # because 1/-1 corresponds to pi/-pi radians rotation
        scaled += self.sim_agent.get_rest_pos()
        # this enforces an action range of -1/1, except if it's relative action - then the action space is asymmetric
        clipped = np.clip(scaled, -np.pi + 0.001, np.pi - 0.001)
        return clipped

    def _calc_imitation_error(self, joints_agent, joints_ref):
        diff = np.array(joints_ref - joints_agent)
        pose_err = diff.dot(diff)
        pose_reward = np.exp(-JOINT_ERROR_SCALE * pose_err)
        return pose_reward

    def step(self, action):
        action_clean = self._sanitize_actions(action)

        ##  reference sim
        self.frame_idx += 1  # retrieving the next recording frame

        joints_reference = get_recording_joints(self.joints, self.frame_idx)
        self.sim_ref.set(joints_reference)

        pos, rot = get_recording_pose(self.vicon_positions, self.base_pos, self.frame_idx)
        self.sim_ref.move_kinectic_body(pos, rot.as_quat())

        self.sim_ref.step()

        ## learner sim
        self.sim_agent.action(action_clean)

        for _ in range(self.sim_steps):
            self.sim_agent.step()

        joints_agent = self.sim_agent.get_joint_states()

        obs = self._get_obs()
        reward = self._calc_imitation_error(joints_agent, joints_reference)
#         print (reward)
        done = False
        misc = {}

        if self.frame_idx == self.idx_end:
            done = True

        return obs, reward, done, misc

    def _render(self, target_sim):
        with_segmap = False
        if MASKED:
            with_segmap = True

        img, segmap = getattr(self, target_sim).take_photo(
            with_segmap=with_segmap, camera_offset=CAMERA_OFFSET, lookat_offset=LOOKAT_OFFSET
        )

        if MASKED:
            img[segmap != 1] = 0
        return img

    def getVisualState(self, mode='rgb_array'):
        img = self._render_agent()
#         print ("img.shape: ", img.shape)
        img = np.mean(img, axis=-1, keepdims=True)
        pos, orn, vel = self.sim_agent.get_pos_orn_vel()
        img = np.concatenate((img.flatten(), vel), axis=-1) ## to grayscale
        return img
    
    def getImitationVisualState(self, mode='rgb_array'):
        img = self._render_ref()
        img = np.mean(img, axis=-1, keepdims=True)
        pos, orn, vel = self.sim_ref.get_pos_orn_vel()
        img = np.concatenate((img.flatten(), vel), axis=-1) ## to grayscale
        return img
    
    def _render_agent(self):
        return self._render("sim_agent")

    def _render_ref(self):
        return self._render("sim_ref")

    def render(self, mode="human"):
        mode_i = rendermode_from_string(mode)

        if mode_i is RenderMode.HUMAN:
            img_agent = self._render_agent()
            img_ref = self._render_ref()
            cv2.imshow("Left: Ref, Right: Agent", merge_images(img_ref, img_agent))
            cv2.waitKey(1)

        elif mode_i is RenderMode.RGB_ARRAY:
            return self._render_agent()
        elif mode_i is RenderMode.RGB_ARRAY_REF:
            return self._render_ref()
        else:
            raise NotImplementedError(f"I don't have a render function for mode {mode_i}.")


if __name__ == "__main__":

    # necessary for gym env to register (not here but when you copy this snippet over)
    import stanford_quad

    env = gym.make("Pupper-Recording-WalkForward-v0")

    for _ in range(3):
        obs = env.reset()
        print(obs)
        while True:
            obs, rew, done, misc = env.step(np.random.uniform(-0.1, 0.1, 12))
            print(rew, obs)
            env.render("human")
            if done:
                break
