from gym import register
import numpy as np

from stanford_quad.envs.imitation_recordings import IMITATION_LIB

HEADLESSNESS = ["Headless", "Graphical"]


def make_imitation_env(trick):
    steps = IMITATION_LIB[trick]["end"] - IMITATION_LIB[trick]["start"]
    register(
        id=f"Pupper-Recording-{IMITATION_LIB[trick]['env_name']}-v0",
        entry_point="stanford_quad.envs:ImitationEnv",
        kwargs={
            # "debug": (False if headlessness == "Headless" else True),
            "trick": "walk-forward",
        },
        max_episode_steps=steps,
    )


make_imitation_env("walk-forward")


ACTION_TYPE = ["Relative", "Absolute", "Incremental"]
# ACTION_SMOOTHING = [1, 2, 3, 4]
RANDOM_ROT = [0, 1, 10]
ACTION_SCALING = [0.1, 0.5, 1.0, 2.0]
GAIT_FACTOR = [0.0, 0.33, 0.66, 0.95]
CTRL_FREQ = [60, 10]
SECONDS_OF_SIM = 2

for headlessness in HEADLESSNESS:
    for ctrl_freq in CTRL_FREQ:
        for action_type in ACTION_TYPE:
            # for action_smoothing in ACTION_SMOOTHING:
            for action_scaling in ACTION_SCALING:
                for random_rot in RANDOM_ROT:
                    for gait_fact in GAIT_FACTOR:
                        name = (
                            f"Pupper-Walk-{action_type}-"
                            f"aScale_{action_scaling:.2}-"
                            # f"aSmooth_{action_smoothing}-"
                            f"freq_{ctrl_freq}-"
                            f"gFact_{gait_fact}-"
                            f"RandomZRot_{random_rot}-{headlessness}-v0"
                        )
                        # print(name)
                        register(
                            id=name,
                            entry_point="stanford_quad.envs:WalkingEnv",
                            kwargs={
                                "debug": (False if headlessness == "Headless" else True),
                                "steps": SECONDS_OF_SIM * ctrl_freq,
                                "relative_action": True if action_type == "Relative" else False,
                                "incremental_action": True if action_type == "Incremental" else False,
                                "action_scaling": action_scaling,
                                "action_smoothing": 1,
                                "random_rot": (0, 0, random_rot),
                                "gait_factor": gait_fact,
                                "control_freq": ctrl_freq,
                            },
                            max_episode_steps=SECONDS_OF_SIM * ctrl_freq,
                        )

ctrl_freq = 60

for headlessness in HEADLESSNESS:
    for random_rot in RANDOM_ROT:
        name = (
            f"Pupper-Walk-FRC-"
            # f"aScale_{action_scaling:.2}-"
            f"randomZRot_{random_rot}-{headlessness}-v0"
        )
        # print(name)
        register(
            id=name,
            entry_point="stanford_quad.envs:WalkingEnv",
            kwargs={
                "debug": (False if headlessness == "Headless" else True),
                "steps": SECONDS_OF_SIM * ctrl_freq,
                "random_rot": (0, 0, random_rot),
            },
            max_episode_steps=SECONDS_OF_SIM * ctrl_freq,
        )

# Cyril envs (we can merge and clean later) TODO: Find static parameters.

default_ctrl_freq = 10
default_action_scaling = 0.03
default_gait_fact = 0.5
default_second_of_sim = 50

register(
    id="PupperBaseDebug-v0",
    entry_point="stanford_quad.envs:PupperBase",
    kwargs={
        "debug": True,
        "steps": default_second_of_sim * default_ctrl_freq,
        "relative_action": True,
        "incremental_action": False,
        "action_scaling": default_action_scaling,
        "action_smoothing": 1,
        "random_rot": (0, 0, random_rot),
        "gait_factor": default_gait_fact,
        "control_freq": default_ctrl_freq,
    },
    max_episode_steps=default_second_of_sim * default_ctrl_freq,
)