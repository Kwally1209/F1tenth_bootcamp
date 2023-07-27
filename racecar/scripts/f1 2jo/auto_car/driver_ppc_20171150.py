###########################################################
#############  MEN491 Creating Autonomous Car #############
#############     PURE PURSUIT ASSIGNMENT     #############
#############     20171150    WooJae Shin     #############
###########################################################

# ASSIGNMENT - Implement the driver using Pure Pursuit algorithm 

import numpy as np
import math

class PurePursuitDriver:

    def __init__(self):
        self.prev_idx = 0

    def pure_pursuit_control(self, pose_x, pose_y, pose_theta, ref):
        
        # TODO implement pure pursuit algorithm

        # Set the lookahead distance
        L_f = 4 #ahead distance

        # Find the lookahead point on the reference trajectory
        dist = np.abs((ref[:,0]-pose_x)**2+(ref[:,1]-pose_y)**2-L_f**2)

        idx = np.argmin(dist)

        if idx > self.prev_idx and idx-self.prev_idx < 10:
            self.prev_idx = idx
        
        lookahead_idx = self.prev_idx
        
        # Compute the heading to the lookahead point
        theta = math.atan2(ref[lookahead_idx, 1]-pose_y, ref[lookahead_idx, 0]-pose_x)-pose_theta

        theta = math.atan2(math.sin(theta), math.cos(theta))

        # Compute the steering angle
        steering_angle = math.atan2(2.0*math.sin(theta), L_f)

        # Compute the speed (you may use constant speed simply)
        speed = 10.5
        #speed = 17.4 - 50*np.abs(steering_angle) #35.56
        #if speed < 5:
        #    speed = 5

        return speed, steering_angle, lookahead_idx


