import pybullet as p
cin = p.connect(p.SHARED_MEMORY)
if (cin < 0):
    cin = p.connect(p.GUI)
objects = [p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("quadruped/minitaur_v1.urdf", 1.000397,-0.990977,0.166281,-0.001128,0.001265,0.195322,0.980738)]
ob = objects[0]
jointPositions=[ 0.000000, 1.569646, 0.000000, -2.182872, 1.570201, 0.000000, -2.191865, 1.569370, 0.000000, -2.182832, 1.570121, 0.000000, -2.187810, 0.000000, -1.570037, 0.000000, 2.182983, -1.569349, 0.000000, 2.201204, -1.568805, 0.000000, 2.182471, -1.568983, 0.000000, 2.200243 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

cid0 = p.createConstraint(1,19,1,16,p.JOINT_POINT2POINT,[0.000000,0.000000,0.000000],[0.000000,0.005000,0.100000],[0.000000,0.010000,0.100000],[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
p.changeConstraint(cid0,maxForce=1000.000000)
cid1 = p.createConstraint(1,3,1,6,p.JOINT_POINT2POINT,[0.000000,0.000000,0.000000],[0.000000,0.005000,0.100000],[0.000000,0.010000,0.100000],[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
p.changeConstraint(cid1,maxForce=1000.000000)
cid2 = p.createConstraint(1,25,1,22,p.JOINT_POINT2POINT,[0.000000,0.000000,0.000000],[0.000000,0.005000,0.100000],[0.000000,0.010000,0.100000],[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
p.changeConstraint(cid2,maxForce=1000.000000)
cid3 = p.createConstraint(1,9,1,12,p.JOINT_POINT2POINT,[0.000000,0.000000,0.000000],[0.000000,0.005000,0.100000],[0.000000,0.010000,0.100000],[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
p.changeConstraint(cid3,maxForce=1000.000000)
p.setGravity(0.000000,0.000000,-10.000000)
p.stepSimulation()
p.disconnect()
