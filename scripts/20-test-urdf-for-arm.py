import math
import pybullet as p
import time
import pybullet_data
import numpy as np

# Create the bullet physics engine environment
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.resetDebugVisualizerCamera(cameraDistance=0.45, cameraYaw=135, cameraPitch=-45, cameraTargetPosition=[0, 0, 0])
p.setGravity(0, 0, -10)  # good enough
frequency = 240  # Hz
p.setTimeStep(1 / frequency)
p.setRealTimeSimulation(0)

# This loads the checkerboard background
p.loadURDF("plane.urdf")

# Robot model starting position
startPos = [0, 0, 0]  # xyz
startOrientation = p.getQuaternionFromEuler([0, 0, 0])  # rotated around which axis? # np.deg2rad(90)

# Actually load the URDF file into simulation, make the base of the robot unmoving
robot = p.loadURDF("./ergourdf.urdf.xml", startPos, startOrientation, useFixedBase=1)

for i in range(p.getNumJoints(robot)):
    print(p.getJointInfo(robot, i))

motors = [0, 1, 2]

# Container for debug inputs
debugParams = []

# In the user interface, create a slider for each motor to control them separately.
for i in range(len(motors)):
    motor = p.addUserDebugParameter("motor{}".format(i + 1), -1, 1, 0)
    debugParams.append(motor)

start = time.time()

# Stepping frequency * 30 = run the simulation for 30 seconds
for i in range(frequency * 30):
    motorPos = []

    for i in range(len(motors)):
        pos = (math.pi / 2) * p.readUserDebugParameter(debugParams[i])
        motorPos.append(pos)
        p.setJointMotorControl2(robot, motors[i], p.POSITION_CONTROL, targetPosition=pos)

    p.stepSimulation()

    time.sleep(1.0 / frequency)

print(time.time() - start)

p.disconnect()
