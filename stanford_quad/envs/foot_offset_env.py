from collections import deque

import gym
from gym import spaces
import numpy as np

from stanford_quad.assets import ASSET_DIR
from stanford_quad.common.Utilities import controller_to_sim
from stanford_quad.pupper.policy import HardPolicy
from stanford_quad.sim.simulator2 import PupperSim2, FREQ_SIM

CONTROL_FREQUENCY = 60  # Hz, the simulation runs at 240Hz by default and doing a multiple of that is easier
MAX_ANGLE_PER_SEC = 90


class FootOffsetEnv(gym.Env):
    def __init__(
        self,
        debug=False,
        steps=120,
        control_freq=CONTROL_FREQUENCY,
        action_scaling=1.0,
        random_rot=(0, 0, 0),
        reward_coefficients=(0.1, 1, 1),
        stop_on_flip=False,
    ):
        """ Gym-compatible environment to teach the pupper how to walk - with predefined gait as baseline
        """

        super().__init__()

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

        self.observation_space = spaces.Box(low=-1, high=1, shape=(12 + 3 + 2,), dtype=np.float32)

        # action = x/y/z offset for all 4 feet
        self.action_space = spaces.Box(low=-1, high=1, shape=(12,), dtype=np.float32)

        # turning off start_standing because the that's done in self.reset()
        kwargs = {}
        # if relative_action: # if this is turned on, the robot is underpowered for the hardcoded gait
        #     kwargs = {"gain_pos": 1 / 16, "gain_vel": 1 / 8, "max_torque": 1 / 2}

        self.sim = PupperSim2(debug=debug, start_standing=False, **kwargs)
        self.episode_steps = 0
        self.episode_steps_max = steps
        self.control_freq = control_freq
        self.dt = 1 / self.control_freq
        self.incremental_angle = np.deg2rad(MAX_ANGLE_PER_SEC) / self.control_freq
        self.sim_steps = int(round(FREQ_SIM / control_freq))
        print(f"Running with {self.sim_steps} substeps")
        self.action_scaling = action_scaling
        self.random_rot = random_rot
        self.stop_on_flip = stop_on_flip
        self.current_action = np.array([0] * 12)
        self.gait = None
        ranges = np.load(f"{ASSET_DIR}/joint_foot_ranges.npz")
        self.foot_ranges = ranges["foot_ranges"].reshape(12, 2)
        self.foot_ranges_diff = self.foot_ranges[:, 1] - self.foot_ranges[:, 0]
        # extend the allowed foot ranges by 25% of the range on that axis
        print("fr before", self.foot_ranges)
        self.foot_ranges = np.array(
            [
                (fr[0] - 0.25 * fd, fr[1] + 0.25 * fd) if fd > 0 else (fr[0] - 0.1, fr[1] + 0.1)
                for fr, fd in zip(self.foot_ranges, self.foot_ranges_diff)
            ]
        )
        self.foot_ranges_diff[self.foot_ranges_diff == 0] = 0.2  # corresponding to min dist
        print("fr after", self.foot_ranges)
        self.joint_ranges = ranges["joint_ranges"]
        print("joint ranges", self.joint_ranges)

        # new reward coefficients
        self.rcoeff_ctrl, self.rcoeff_run, self.rcoeff_stable = reward_coefficients

    def reset(self):
        self.episode_steps = 0
        self.sim.reset(rest=True)  # also stand up the robot

        self.gait = HardPolicy()

        return self.get_obs()

    def seed(self, seed=None):
        np.random.seed(seed)
        return super().seed(seed)

    def close(self):
        self.sim.p.disconnect()
        super().close()

    def sanitize_actions(self, actions):
        assert len(actions) == 12
        actions_clipped = (np.clip(actions, -1, 1).astype(np.float32) + 1) / 2

        # denormalize actions to correspond to foot offset
        actions_denorm = actions_clipped * self.foot_ranges_diff + self.foot_ranges[:, 0]

        # stepping the gait to get the foot positions
        self.gait.act(velocity_horizontal=(0.2, 0), normalized=False)

        # combine the gait with the offset, half-half
        actions_merged = 0.5 * actions_denorm + 0.5 * self.gait.state.foot_locations.T.flatten()

        # make sure feet don't exceed the foot max range
        clipped_feet = np.clip(actions_merged, self.foot_ranges[:, 0], self.foot_ranges[:, 1])

        # reshape and apply inverse kinematics to get joints from feet
        clipped_feet = clipped_feet.reshape(4, 3).T
        joints = self.gait.controller.inverse_kinematics(clipped_feet, self.gait.controller.config)

        # convert joints from 3,4 repr to sim-usable one
        joints = controller_to_sim(joints)

        return joints

    def get_obs(self):
        pos, orn, vel = self.sim.get_pos_orn_vel()

        joint_states = np.array(self.sim.get_joint_states())
        # per-joint normalization based on the max useful range
        joint_states = np.array(
            [(js - rngs[0]) / (rngs[1] - rngs[0]) for js, rngs in zip(joint_states, self.joint_ranges)]
        ) / (np.pi * 1.25)

        obs = list(joint_states) + list(orn) + list(vel)[:2]
        return obs

    def step(self, action):
        action_clean = self.sanitize_actions(action)

        pos_before, _, _ = self.sim.get_pos_orn_vel()

        self.sim.action(action_clean)

        for _ in range(self.sim_steps):
            self.sim.step()

        pos_after, orn_after, _ = self.sim.get_pos_orn_vel()

        obs = self.get_obs()

        # this reward calculation is taken verbatim from halfcheetah-v2, save
        reward_ctrl = -np.square(action).sum()
        reward_run = (pos_after[0] - pos_before[0]) / self.dt
        # technically we should divide the next line by (3*pi^2) but that's really hard to reach
        reward_stable = -np.square(orn_after).sum() / np.square(np.pi)
        reward = self.rcoeff_ctrl * reward_ctrl + self.rcoeff_run * reward_run + self.rcoeff_stable * reward_stable

        done = False
        self.episode_steps += 1
        if self.episode_steps == self.episode_steps_max:
            done = True

        return obs, reward, done, dict(reward_run=reward_run, reward_ctrl=reward_ctrl, reward_stable=reward_stable)

    def render(self, mode="human"):
        # todo: if the mode=="human" then this should open and display a
        #  window a la "cv2.imshow('xxx',img), cv2.waitkey(10)"

        img, _ = self.sim.take_photo()
        return img


if __name__ == "__main__":
    env = FootOffsetEnv(debug=True)

    for _ in range(3):
        env.reset()
        while True:
            # FR, FL, BR, BL
            # _, _, done, _ = env.step([0, 1, 0, 0, -1, 0, 0, 1, 0, 0, -1, 0])
            # _, _, done, _ = env.step([1, 0, 0, 1, 0, 0, -1, 0, 0, -1, 0, 0])
            obs, rew, done, _ = env.step([0] * 12)
            print(rew, done, obs)
            if done:
                break
