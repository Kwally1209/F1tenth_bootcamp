###########################################################
#############  MEN491 Creating Autonomous Car #############
#############     PURE PURSUIT ASSIGNMENT     #############
#############     20201234   Hyeonbin Lee     #############
###########################################################

# ASSIGNMENT - Implement the driver using Pure Pursuit algorithm 

import numpy as np
import math

class PurePursuitDriver:

    def pure_pursuit_control(self, pose_x, pose_y, pose_theta, ref):
        
        # TODO implement pure pursuit algorithm

        # Set the lookahead distance


        # Find the lookahead point on the reference trajectory
        lookahead_idx = 10
               
        # Compute the heading to the lookahead point


        # Compute the steering angle
        steering_angle = 0.0

        # Compute the speed (you may use constant speed simply)
        speed = 5.0

        return speed, steering_angle, lookahead_idx 