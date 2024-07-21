import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from gym.envs.box2d.car_racing import CarRacing
import pygame

from src.lane_detection import LaneDetection
import matplotlib.pyplot as plt
import numpy as np

# action variables
action = np.array([0.0, 0.0, 0.0])
def register_input():

    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                action[0] = -1.0
            if event.key == pygame.K_RIGHT:
                action[0] = +1.0
            if event.key == pygame.K_UP:
                action[1] = +0.5
            if event.key == pygame.K_DOWN:
                action[2] = +0.8  # set 1.0 for wheels to block to zero rotation
            if event.key == pygame.K_r:
                global retry
                retry = True
            if event.key == pygame.K_s:
                global record
                record = True
            if event.key == pygame.K_q:
                global quit
                quit = True

        if event.type == pygame.KEYUP:
            if event.key == pygame.K_LEFT and action[0] < 0.0:
                action[0] = 0
            if event.key == pygame.K_RIGHT and action[0] > 0.0:
                action[0] = 0
            if event.key == pygame.K_UP:
                action[1] = 0
            if event.key == pygame.K_DOWN:
                action[2] = 0

# init environement
env = CarRacing(render_mode="human")
env.render()
env.reset()

# define variables
total_reward = 0.0
steps = 0
restart = False

# init modules of the pipeline
LD_module = LaneDetection()

# init extra plot
fig = plt.figure()
plt.ion()
plt.show()

while True:
    # perform step
    register_input()
    s, r, done, _, info = env.step(action)

    # lane detection
    splines = LD_module.lane_detection(s)
    
    # reward
    total_reward += r

    # outputs during training
    if steps % 2 == 0 or done:
        print("\naction " + str(["{:+0.2f}".format(x) for x in action]))
        print("step {} total_reward {:+0.2f}".format(steps, total_reward))
        LD_module.plot_state_lane(s, steps, fig)
    steps += 1
    env.render()
    
    if done or restart: break

env.close()