import numpy as np
import math

class PurePursuitDriver:

    def __init__(self, lookahead_distance):
        self.lookahead_distance = lookahead_distance

        self.count = 0

    def find_lookahead_idx(self, pose_x, pose_y, ref):
        self.lookahead_distance = 1.0
        distances = np.sqrt((ref[:, 0] - pose_x) ** 2 + (ref[:, 1] - pose_y) ** 2)
        min_idx = np.argmin(distances)
        lookahead_idx = min_idx

        while True:
            lookahead_idx += 1
            if lookahead_idx >= len(ref):
                lookahead_idx = 0
            dist_to_lookahead = np.hypot(ref[lookahead_idx, 0] - pose_x, ref[lookahead_idx, 1] - pose_y)
            if dist_to_lookahead >= self.lookahead_distance:
                break
        
        if (lookahead_idx > 140 or lookahead_idx < 24) and self.count==1:
            self.count=1
            self.lookahead_distance = 0.7
            return find_lookahead_idx(pose_x, pose_y, ref)
            
        self.count=0

        return lookahead_idx

    def pure_pursuit_control(self, pose_x, pose_y, pose_theta, ref):
        # Find the lookahead point on the reference trajectory
        lookahead_idx = self.find_lookahead_idx(pose_x, pose_y, ref)
        lookahead_point = ref[lookahead_idx]

        # Compute the heading to the lookahead point
        target_angle = math.atan2(lookahead_point[1] - pose_y, lookahead_point[0] - pose_x)
        alpha = target_angle - pose_theta

        # Ensure the angle is within the range of [-pi, pi]
        while alpha > np.pi:
            alpha -= 2 * np.pi
        while alpha < -np.pi:
            alpha += 2 * np.pi

        # Compute the steering angle
        steering_angle = (math.atan2(2 * math.sin(alpha), self.lookahead_distance)
        )*0.7
        # Compute the speed (you may use constant speed simply)
        if abs(steering_angle) >0.8:
            speed = 1.5
            # speed = (0.5*9.81*0.35/np.sin(abs(steering_angle)))**0.5
        elif abs(steering_angle)>0.6:
            speed = 2.5
        elif abs(steering_angle)>0.4:
            speed = 3.0
        elif abs(steering_angle)>0.2:
            speed = 4.0
        else:
            speed = 4.1


        

        if lookahead_idx > 130 or lookahead_idx < 24:
            speed = speed/2.3

        if lookahead_idx >74 and lookahead_idx <131:
            speed = speed/1.8

        if lookahead_idx >70 and lookahead_idx <130:
            steering_angle = np.clip(steering_angle,-0.2,0.2)
        # speed = (0.1*9.81*0.35/np.sin(abs(steering_angle)))**0.5
        print(speed)
        return speed, steering_angle, lookahead_idx



