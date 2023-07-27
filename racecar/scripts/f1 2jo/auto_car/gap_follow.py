#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import itertools,operator

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

prev_steering_angle = 0
t_prev = 0.0
prev_range = []

view_angel = 180 #0-135
view_idx = int(1077/135*view_angel)

Kp = 1
Ki = 1
Kd = 1

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        self.lidar_sub = rospy.Subscriber('/scan',LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd',AckermannDriveStamped,queue_size=5)

    def find_max_gap(self, free_space_ranges):
       # mask the bubble
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        print(free_space_ranges)
        # get a slice for each contigous sequence of non-bubble data
        slices = np.ma.notmasked_contiguous(masked)
        chosen_slice = max(slices, key = lambda x : x.stop - x.start)
        
        return chosen_slice.start, chosen_slice.stop


 
        

    def find_best_point(self, start_i, end_i, ranges):
        print(start_i)      
        return  (end_i + start_i)/2
    #np.argmax(ranges[start_i : end_i])

    def lidar_callback(self, data):

        print("@@@@@@@@@@@@@@@@@@222")
        
        # Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        #TO DO:  
        # Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        #       1. Get LiDAR message
        #       2. Find closest point to LiDAR
        #       3. Eliminate all points inside 'bubble' (set them to zero) 
        #       4. Fine the max gap -> "def find_max_gap(self, free_space_ranges):"
        #       4. Find the best point -> "def find_best_point(self, start_i, end_i, ranges):"
        #       5. Publish Drive message

        # Get LiDAR message

        
        ranges = np.array(data.ranges)[1077-int(view_idx/2):1078+int(view_idx/2)]
        # angles = np.arange(data.angle_min, data.angle_max+data.angle_increment, data.angle_increment)
        # angles = np.arange()

        # print(angles)

        # Find closest point to LiDAR
        closest_range = np.min(ranges)
        print("CR :{}".format(closest_range))

        closest_i = np.argmin(ranges)
        print("CI :{}".format(closest_i))

        # Eliminate all points inside 'bubble' (set them to zero) 
        bubble_radius = 500 # index //meters
        # bubble_indices = np.logical_and(
        #     ranges > 0,
        #     ranges < bubble_radius
        # )

        min_idx=closest_i - bubble_radius
        max_idx=closest_i + bubble_radius
        
        if min_idx <0 : min_idx = 0
        if max_idx >= len(ranges): max_idx = len(ranges)-1
        ranges[min_idx:max_idx] =0
        # ranges[closest_i-bubble_radius/2:closest_i+bubble_radius/2] = 0

        # Find max gap and best point within gap
        start_i, end_i = self.find_max_gap(ranges)
        print(start_i, end_i)
        if start_i == None and end_i == None:
            start_i = 0

        best_i = self.find_best_point(start_i, end_i, ranges)

        print(best_i)
        print("$$$$$$$$$$")

        # Calculate steering angle
        # steering_angle = angles[best_i]


        steer_gain=0.7
        steering_angle = 0.53*(best_i-view_idx/2+1)/280*steer_gain
        print("steer: {}".format(steering_angle))

        # Publish Drive message
        # global prev_steering_angle, t_prev, prev_range
        if closest_range < 0.2:
            speed = 0.5
        else:
            speed = 1.0

        # print(speed)
        # print("#############################")

        
        # steering_angle = prev_steering_angle + Kp*()


        

        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.stamp = rospy.Time.now()
        self.drive_msg.drive.steering_angle = steering_angle
        self.drive_msg.drive.speed = speed
        self.drive_pub.publish(self.drive_msg)



def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
