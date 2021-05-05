from stanford_quad.sim.simulator2 import PupperSim2

STEP_WIDTH = 2  # if you're standing in front of a staircase and looking up, what's the left-right distance?
STEP_DEPTH = 0.3  # if you're stepping onto one step, how much of your foot can fit on the step?
STEP_HEIGHT = 0.05  # how far for you have to go up between each step?
OFFSET = (0.3, 0, 0)

sim = PupperSim2(debug=True)
sim.reset()
sim.add_rooms()

for step in range(100000):
    # sim.action(motorPo)
    sim.step()
