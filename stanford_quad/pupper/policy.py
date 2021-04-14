import numpy as np

from stanford_quad.common.Command import Command
from stanford_quad.common.Controller import Controller
from stanford_quad.common.ControllerArm import ControllerArm
from stanford_quad.common.State import State
from stanford_quad.pupper.Config import Configuration
from stanford_quad.pupper.Kinematics import four_legs_inverse_kinematics


class HardPolicy:
    def __init__(self, fps=60, warmup=0, dbg=False) -> None:
        super().__init__()

        self.fps = fps
        self.warmup = warmup

        self.config = Configuration()
        self.config.dt = 1 / fps
        # config.z_clearance = 0.02
        self.controller = Controller(self.config, four_legs_inverse_kinematics)
        self.controller_arm = ControllerArm(self.config)
        self.state = State()
        command = Command()

        # initializing the controller
        command.activate_event = 1
        self.controller.run(self.state, command)
        command.activate_event = 0
        command.trot_event = 1
        self.controller.run(self.state, command)
        self.command = Command()  # zero it out

        if dbg:
            print("Summary of gait parameters:")
            print("overlap time: ", self.config.overlap_time)
            print("swing time: ", self.config.swing_time)
            print("z clearance: ", self.config.z_clearance)
            print("x shift: ", self.config.x_shift)
        self.step = 0

    def _sanity_check_actions(self, velocity, yaw, arm):
        assert len(velocity) == 2
        assert len(arm) == 3
        if np.abs(velocity[0]) > self.config.max_x_velocity:
            print(f"WARNING: x velocity {velocity[0]} exceeding maximum {self.config.max_x_velocity}. Clipping.")
        if np.abs(velocity[1]) > self.config.max_y_velocity:
            print(f"WARNING: y velocity {velocity[1]} exceeding maximum {self.config.max_y_velocity}. Clipping.")
        if np.abs(yaw) > self.config.max_yaw_rate:
            print(f"WARNING: yaw rate {yaw} exceeding maximum {self.config.max_yaw_rate}. Clipping.")
        velocity = np.clip(
            velocity,
            [-self.config.max_x_velocity, -self.config.max_y_velocity],
            [self.config.max_x_velocity, self.config.max_y_velocity],
        )
        yaw = np.clip(yaw, -self.config.max_yaw_rate, self.config.max_yaw_rate)
        arm = np.clip(arm, -self.config.max_arm_rate, self.config.max_arm_rate)
        return velocity, yaw, arm

    def act(self, velocity_horizontal=(0, 0), yaw_rate=0, velocity_arm=(0, 0, 0), normalized=False):
        velocity, yaw, arm = self._sanity_check_actions(velocity_horizontal, yaw_rate, velocity_arm)

        if self.step > self.warmup:
            self.command.horizontal_velocity = np.array(velocity)
            self.command.yaw_rate = yaw
            self.command.arm_diff = arm
        else:
            self.command.horizontal_velocity = np.array([0, 0])

        self.state.quat_orientation = np.array([1, 0, 0, 0])

        # Step the controller forward by dt
        self.controller.run(self.state, self.command)
        self.controller_arm.run(self.state, self.command, normalized)

        if normalized:
            self.state.joint_angles /= np.pi

        self.step += 1

        return self.state.joint_angles, self.state.arm_joints
