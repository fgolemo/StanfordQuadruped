from collections import deque

import numpy as np

from stanford_quad.common.ArmIK import ArmIK
from stanford_quad.common.Command import Command
from stanford_quad.common.State import State
from stanford_quad.pupper import Config


class ControllerArm:
    """Controller and planner object
    """

    def __init__(self, config: Config):
        self.config = config
        self.ik = ArmIK()
        self.last_tick = -np.inf
        self.buffer = deque(maxlen=1 + self.config.arm_smooth)

    # def step_gait(self, state, command):
    #     """Calculate the desired arm locations for the next timestep
    #
    #     Returns
    #     -------
    #     Numpy array (3, 4)
    #         Matrix of new foot locations.
    #     """
    #
    #     return new_foot_locations, contact_modes

    def run(self, state: State, command: Command, normalized: bool = True):
        """Steps the controller forward one timestep
        """
        state.arm_pos += command.arm_diff
        if np.count_nonzero(command.arm_diff) > 0 and state.ticks - self.last_tick > self.config.arm_ik_ticks:
            state.arm_joints = np.array(self.ik.pos2joints(state.arm_pos))
            self.last_tick = np.copy(state.ticks)
            if normalized:
                state.arm_joints = np.deg2rad(state.arm_joints)

            # moving avg smoothing
            self.buffer.append(np.copy(state.arm_joints))
            state.arm_joints = np.mean(self.buffer, axis=0)
