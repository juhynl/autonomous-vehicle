import argparse
import numpy as np

from gym.envs.box2d.car_racing import CarRacing
from lane_detection import LaneDetection
from waypoint_prediction import waypoint_prediction, target_speed_prediction
from lateral_control import LateralController
from longitudinal_control import LongitudinalController


def evaluate(env):
    """
    """

    # action variables
    a = np.array( [0.0, 0.0, 0.0] )

    # init environement
    env.render()
    env.reset()


    for episode in range(5):
        observation = env.reset()
        # init modules of the pipeline
        LD_module = LaneDetection()
        LatC_module = LateralController()
        LongC_module = LongitudinalController()
        reward_per_episode = 0
        for t in range(500):
            # perform step
            #out = env.step(a)
            #s, r, done, info = env.step(a)
            #speed = info['speed']
            velocity = env.car.hull.linearVelocity
            s, r, done, tmp, info = env.step(a)
            speed = np.sqrt(np.square(velocity[0]) + np.square(velocity[1]))

            # lane detection
            lane1, lane2 = LD_module.lane_detection(s)
            #print('lane1, lane2 ',lane1, lane2)

            # waypoint and target_speed prediction
            waypoints = waypoint_prediction(lane1, lane2)
            target_speed = target_speed_prediction(waypoints, max_speed=60, exp_constant=4.5)

            # control
            a[0] = LatC_module.stanley(waypoints, speed)
            a[1], a[2] = LongC_module.control(speed, target_speed)

            # reward
            reward_per_episode += r
            env.render()

        print('episode %d \t reward %f' % (episode, reward_per_episode))



def calculate_score_for_leaderboard(env):
    """
    Evaluate the performance of the network. This is the function to be used for
    the final ranking on the course-wide leader-board, only with a different set
    of seeds. Better not change it.
    """
    # action variables
    a = np.array( [0.0, 0.0, 0.0] )

    # init environement
    env.render()
    env.reset()

    seeds = [22597174, 68545857, 75568192, 91140053, 86018367,
             49636746, 66759182, 91294619, 84274995, 31531469]


    total_reward = 0

    for episode in range(10):
        #env.seed(seeds[episode])
        #observation = env.reset()
        env.reset(seed=seeds[episode])

        # init modules of the pipeline
        LD_module = LaneDetection()
        LatC_module = LateralController()
        LongC_module = LongitudinalController(KP=0.03, KI=0, KD=0.05)

        reward_per_episode = 0
        for t in range(600):
            # perform step
            #s, r, done, speed, info = env.step(a)
            s, r, done, tmp, info = env.step(a)
            velocity = env.car.hull.linearVelocity
            speed = np.sqrt(np.square(velocity[0]) + np.square(velocity[1]))

            # lane detection
            lane1, lane2 = LD_module.lane_detection(s)

            # waypoint and target_speed prediction
            waypoints = waypoint_prediction(lane1, lane2)
            target_speed = target_speed_prediction(waypoints, max_speed=80, exp_constant=15, offset_speed=20)

            # control
            a[0] = LatC_module.stanley(waypoints, speed)
            a[1], a[2] = LongC_module.control(speed, target_speed)

            # reward
            reward_per_episode += r

            env.render()

        print('episode %d \t reward %f' % (episode, reward_per_episode))
        total_reward += np.clip(reward_per_episode, 0, np.infty)

    print('---------------------------')
    print(' total score: %f' % (total_reward / 10))
    print('---------------------------')

if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument( '--score', action="store_true", help='a flag to evaluate the pipeline for the leaderboard' )
    parser.add_argument( '--display', action="store_true", help='a flag indicating whether training runs in the cluster' )

    args = parser.parse_args()

    

    if args.score:
        env = CarRacing(render_mode="rgb_array")
        calculate_score_for_leaderboard(env)
    else:
        env = CarRacing(render_mode="human")
        evaluate(env)

    env.close()

