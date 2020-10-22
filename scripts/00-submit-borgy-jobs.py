import os
import subprocess
from tqdm import tqdm

base_cmd = """(export SEEDBASE={sb}; export EXP={exp_no}; for ((i = 0; i < {no_seeds}; i++)); do borgy submit -i images.borgy.elementai.net/fgolemo/gym:v4 --mem 32 --gpu-mem 12 --gpu 1 --cuda-version 10.0 -H -- zsh -c "cd /root && source ./.zshrc && export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/mnt/home/fgolemo/.mujoco/mujoco200/bin && cd ~/dev/StanfordQuadruped/ && pip install -e '.[sim]' && cd ~/dev/pytorch-a2c-ppo-acktr-gail && pip install --upgrade numpy && pip install wandb moviepy imageio efficientnet-pytorch pybullet && python main.py --custom-gym 'stanford_quad' --env-name '{env}' --algo ppo --use-gae --log-interval 1 --num-steps 2040 --num-processes 1 --lr 3e-4 --entropy-coef 0 --value-loss-coef 0.5 --ppo-epoch 10 --num-mini-batch 32 --gamma 0.99 --gae-lambda 0.95 --frame-stacc {frame_stacc} --num-env-steps {env_steps} --use-linear-lr-decay --wandb pupper-walk2 --save-interval 10 --gif-interval 50 --seed $((i+SEEDBASE)) > ~/borgy/$((EXP))-pupperwalk-exp$((EXP))-$((i+SEEDBASE)).log"; done)"""

with open("../list_of_environments.txt", "r") as f:
    lines = f.readlines()

lines = [line[:-1] if "\n" in line else line for line in lines]

EXP_NO = 52
NO_SEEDS = 3
ENV_STEPS = 1e6
SB = 0
jobs = 0

for frame_stacc in [1, 4]:
    for line in tqdm(lines):
        cmd = base_cmd.format(
            sb=SB, exp_no=EXP_NO, no_seeds=NO_SEEDS, env=line, frame_stacc=frame_stacc, env_steps=int(ENV_STEPS)
        )
        # print(cmd)
        subprocess.call(cmd, shell=True, executable="/bin/bash")
        # quit()
        jobs += NO_SEEDS
        SB += NO_SEEDS

print(f"\n\nsubmitted {jobs} total")
