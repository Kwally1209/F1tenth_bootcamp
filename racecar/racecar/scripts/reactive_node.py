#!/usr/bin/env python

import rospy
import numpy as np
import std_msgs.msg
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class Reactive:

    def __init__(self,environment= "real"):
        self.CAR_WIDTH = 0.31
        # the min difference between adjacent LiDAR points for us to call them disparate
        self.DIFFERENCE_THRESHOLD = 2.
        n_rate = rospy.get_param('~rate', default=20)
        self.SPEED = rospy.get_param('~vel', default=2.0)
        self.vel = 0.5 
        # the extra safety room we plan for along walls (as a percentage of car_width/2)
        self.SAFETY_PERCENTAGE = 200.
        
        if environment == "sim":                # Simulation environment         
            control_topic = "/opp_id/drive"         
            state_topic = "/opp_id/odom"
            lidar_topic = "/opp_id/scan"
            self.frame = "opp_racecar/laser"  
            # print("sim env")
        else:                                   # Real world setup
            control_topic = "/vesc/low_level/ackermann_cmd_mux/input/gap"
            state_topic = "/pf/pose/odom"
            lidar_topic = "/scan"            
            self.frame = "laser"
            # print("real env")

        pursuit_topic = "control_target"
                
        self.control_pub = rospy.Publisher(control_topic, AckermannDriveStamped, queue_size=1)
        self.ref_pub = rospy.Publisher(pursuit_topic, Marker, queue_size=1)

        self.laser_sub = rospy.Subscriber(lidar_topic, LaserScan, self.process_lidar)
        self.state_sub = rospy.Subscriber(state_topic, Odometry, self.process_state)

        rate= rospy.Rate(n_rate)
        while not rospy.is_shutdown():
            rate.sleep()

    def process_state(self,msg):
        # pose = msg.pose.pose
        # self.pos = np.array([pose.position.x,pose.position.y])
        self.vel = np.clip(msg.twist.twist.linear.x, self.SPEED - 0.5, self.SPEED + 0.5)
                  
    def preprocess_lidar(self, ranges):
        """ Any preprocessing of the LiDAR data can be done in this function.
            Possible Improvements: smoothing of outliers in the data and placing
            a cap on the maximum distance a point can be.
        """
        # remove quadrant of LiDAR directly behind us
        eighth = int(len(ranges) / 8)
        return np.array(ranges[eighth:-eighth])

    def get_differences(self, ranges):
        """ Gets the absolute difference between adjacent elements in
            in the LiDAR data and returns them in an array.
            Possible Improvements: replace for loop with numpy array arithmetic
        """
        differences = [0.]  # set first element to 0
        for i in range(1, len(ranges)):
            differences.append(abs(ranges[i] - ranges[i - 1]))
        return differences

    def get_disparities(self, differences, threshold):
        """ Gets the indexes of the LiDAR points that were greatly
            different to their adjacent point.
            Possible Improvements: replace for loop with numpy array arithmetic
        """
        disparities = []
        for index, difference in enumerate(differences):
            if difference > threshold:
                disparities.append(index)
        return disparities

    def get_num_points_to_cover(self, dist, width):
        """ Returns the number of LiDAR points that correspond to a width at
            a given distance.
            We calculate the angle that would span the width at this distance,
            then convert this angle to the number of LiDAR points that
            span this angle.
            Current math for angle:
                sin(angle/2) = (w/2)/d) = w/2d
                angle/2 = sininv(w/2d)
                angle = 2sininv(w/2d)
                where w is the width to cover, and d is the distance to the close
                point.
            Possible Improvements: use a different method to calculate the angle
        """
        angle = 2 * np.arcsin(width / (2 * dist))
        num_points = int(np.ceil(angle / self.radians_per_point))
        return num_points

    def cover_points(self, num_points, start_idx, cover_right, ranges):
        """ 'covers' a number of LiDAR points with the distance of a closer
            LiDAR point, to avoid us crashing with the corner of the car.
            num_points: the number of points to cover
            start_idx: the LiDAR point we are using as our distance
            cover_right: True/False, decides whether we cover the points to
                         right or to the left of start_idx
            ranges: the LiDAR points

            Possible improvements: reduce this function to fewer lines
        """
        new_dist = ranges[start_idx]
        if cover_right:
            for i in range(num_points):
                next_idx = start_idx + 1 + i
                if next_idx >= len(ranges): break
                if ranges[next_idx] > new_dist:
                    ranges[next_idx] = new_dist
        else:
            for i in range(num_points):
                next_idx = start_idx - 1 - i
                if next_idx < 0: break
                if ranges[next_idx] > new_dist:
                    ranges[next_idx] = new_dist
        return ranges

    def extend_disparities(self, disparities, ranges, car_width, extra_pct):
        """ For each pair of points we have decided a large difference between them, we choose which side to cover (the opposite to the closer point), 
						call the cover function, and return the resultant covered array.
            Possible Improvements: reduce to fewer lines
        """
        width_to_cover = (car_width / 2) * (1 + extra_pct / 100)
        for index in disparities:
            first_idx = index - 1
            points = ranges[first_idx:first_idx + 2]
            close_idx = first_idx + np.argmin(points)
            far_idx = first_idx + np.argmax(points)
            close_dist = ranges[close_idx]
            close_dist = np.clip(close_dist, 0.15,30)
            num_points_to_cover = self.get_num_points_to_cover(close_dist, width_to_cover)
            cover_right = close_idx < far_idx
            ranges = self.cover_points(num_points_to_cover, close_idx, cover_right, ranges)
        return ranges

    def process_lidar(self, msg):
        """ Run the disparity extender algorithm!
            Possible improvements: varying the speed based on the
            steering angle or the distance to the farthest point.
        """
        proc_ranges = np.array(msg.ranges)
        self.radians_per_point = msg.angle_increment #(abs(msg.angle_min) + abs(msg.angle_max) )/ len(ranges)
        
        try:
            differences = self.get_differences(proc_ranges)
            disparities = self.get_disparities(differences, self.DIFFERENCE_THRESHOLD)
            proc_ranges = self.extend_disparities(disparities, proc_ranges, self.CAR_WIDTH, self.SAFETY_PERCENTAGE)
            lidar_angle = (proc_ranges.argmax() - (len(proc_ranges) / 2)) * self.radians_per_point
            # lidar_angle = np.arctan2(2*np.sin(lidar_angle),self.vel)
            steering_angle = np.clip(lidar_angle, -0.45,0.45)
        except:
            lidar_angle = (proc_ranges.argmin() - (len(proc_ranges) / 2)) * self.radians_per_point
            steering_angle = np.clip(-lidar_angle, -0.45,0.45)


        
        next_control = AckermannDriveStamped()
        next_control.header = std_msgs.msg.Header()
        next_control.header.stamp = rospy.Time.now()
        next_control.header.frame_id = "reactive"
        next_control.drive.steering_angle = steering_angle 
        next_control.drive.speed = self.vel
        
        # print("control input : ", [np.round(self.vel,4), np.round(steering_angle,4)]) 
        self.control_pub.publish(next_control)  
                
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "reactive"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD               
        marker.pose.position.x = self.vel*np.cos(steering_angle) 
        marker.pose.position.y = self.vel*np.sin(steering_angle) 
        marker.pose.position.z = 0
        marker.color.r, marker.color.g, marker.color.b = (0, 255, 0)
        marker.color.a = 1
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        self.ref_pub.publish(marker)
        

def main():
    rospy.init_node("reactive node")
    env = rospy.get_param('~Environment', default='real')
    Reactive(env)

if __name__ == "__main__":
    main()
    