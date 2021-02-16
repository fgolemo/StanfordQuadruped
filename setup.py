from setuptools import setup

setup(
    name="gym_pupper",
    version="1.0",
    install_requires=["tqdm", "numpy", "gym", "transforms3d"],
    extras_require={
        "sim": ["matplotlib", "pybullet", "gym", "opencv-python", "torchvision"],
        "robot": ["UDPComms @ git+https://github.com/stanfordroboticsclub/UDPComms@master#egg=UDPComms",],
    },
)
