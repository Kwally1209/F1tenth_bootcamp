###########################################################
########## TEST CODE FOR PURE PURSUIT ASSIGNMENT ##########
###########################################################

# This is the file for testing your pure_pursuit code
# Pure Pursuit Assignment will be graded based on this code
# You can edit this file for debugging
# but final score will be graded by running the original code,
# so please be sure to test with the original file

import time
import gym
import numpy as np
import concurrent.futures
import os
import sys
import matplotlib.pyplot as plt

# Get ./src/ folder & add it to path
current_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.append(current_dir)

# import your drivers here
from drivers_ppc import PurePursuitDriver

# choose your drivers here (1-4)
drivers = [PurePursuitDriver()]

# choose your racetrack here
RACETRACK = 'SILVERSTONE'


class GymRunner(object):

    def __init__(self, racetrack, drivers):
        self.racetrack = racetrack
        self.drivers = drivers
        self.map_points = np.load("./map.npy")
        data = np.genfromtxt('./SILVERSTONE-centerline.csv', delimiter=',', skip_header = 1)
        data[:,0] = data[:,0]-180.8
        data[:,1] = -1*data[:,1]+154.2
        origin = np.argmin(np.abs(data[:,0]))
        self.ref = np.concatenate((data[origin:,:],data[:origin-10,:]),axis=0) 

    def run(self):
        # load map
        env = gym.make('f110_gym:f110-v0',
                       map="{}/maps/{}".format(current_dir, RACETRACK),
                       map_ext=".png", num_agents=len(drivers))

        # specify starting positions of each agent
        poses = np.array([[0. + (i * 0.75), 0. - (i*1.5), np.radians(60)] for i in range(len(drivers))])

        obs, step_reward, done, info = env.reset(poses=poses)
        env.render()

        laptime = 0.0
        checker = 0.0
        check1 = False
        check2 = False
        check3 = False
        check4 = False
        final_point = 0
        start = time.time()

        plt.figure(figsize=(10,20))
        plt.cla()

        while not done:
            actions = []
            futures = []
            with concurrent.futures.ThreadPoolExecutor() as executor:
                for i, driver in enumerate(drivers):
                    futures.append(executor.submit(driver.pure_pursuit_control, obs["poses_x"][0], obs["poses_y"][0], obs["poses_theta"][0], self.ref))
            for future in futures:
                speed, steer, l_idx = future.result()
                actions.append([steer, speed])
            actions = np.array(actions)
            obs, step_reward, done, info = env.step(actions)
            laptime += step_reward
            env.render(mode='human')

            ###########################################################
            ##########      Plot Tools for Debugging        ###########
            ##########  Uncomment lines below to use plot   ###########
            ###########################################################
            # if laptime - checker > 0.1:
            #     checker = laptime
            #     plt.clf()
            #     plt.plot(self.map_points[0,:],self.map_points[1,:],"*k", markersize = 0.5)
            #     plt.plot(self.ref[:,0],self.ref[:,1],"*b", markersize = 1)
            #     plt.plot(self.ref[l_idx,0],self.ref[l_idx,1],"*g", markersize = 5)
            #     plt.plot(obs["poses_x"][0],obs["poses_y"][0],"*r", markersize = 5)
            #     plt.show(block=False)
            #     plt.pause(0.1)

            #################################################
            ############# DO NOT TOUCH THIS CODE#############
            #################################################
            cur_idx = np.argmin(pow((self.ref[:,0] - obs["poses_x"][0]), 2) + pow((self.ref[:,1] - obs["poses_y"][0]), 2))
            if cur_idx > 382 and not check1:
                print("You got 2 points!")
                check1 = True
                final_point = 2
            if cur_idx > 459 and not check2:
                print("You got 3 points!")
                check2 = True
                final_point = 3
            if cur_idx > 628 and not check3:
                print("You got 4 points!")
                check3 = True
                final_point = 4
            if cur_idx == 869 and not check4:
                print("You got 5 points! Congratulations!!")
                check4 = True
                final_point = 5
                break
            if laptime > 60:
                print("Your simulation time exceeds 1 minute. Simulation Done.")

        print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time() - start, 'Final Score :', final_point)

if __name__ == '__main__':
    runner = GymRunner(RACETRACK, drivers)
    runner.run()
