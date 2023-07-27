#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import itertools,operator

from numpy.core.fromnumeric import argmax
from numpy.lib.function_base import vectorize

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive



prev_steering_angle = 0
t_prev = 0.0
prev_range = []

gap_threshold = 10

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        self.lidar_sub = rospy.Subscriber('/scan',LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd',AckermannDriveStamped,queue_size=50)
    	

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        
        # for i in range(0, len(free_space_ranges1)):

        #     if free_space_ranges1[i] > 1:
        #         free_space_ranges1[i] = 1
        #     else:
        #         free_space_ranges1[i] = 0
        
        m = []
        gap_list = []
        count = 0

        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] > 2.6:
                count += 1
                m.append(count)
            else:
                if count > gap_threshold:
                    gap_list.append((i - count, i))
                count = 0
                m.append(count)
                
        for gap in gap_list:
            if gap[1] >=240:
                start_i, end_i = gap
                break

        
        
        # max_idx = np.argmax(m[240:840]) + 240
        # max_num = m[max_idx]
        # end_i = max_idx
        
        # start_i = max_idx - max_num + 1

        # ranges = []
        # for i in range(start_i, end_i):
        #     ranges.append(free_space_ranges[i])
        #     # count = 0
        #     # index = 0
        #     # while free_space_ranges[i] > 0.5:
        #     #     count1 = count + 1
        #     #     count = count1

         
        # print(start_i, end_i)
        return start_i, end_i#, ranges
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
         
        # best_point = np.argmax(ranges[start_i:end_i])
        # best_point = best_point + start_i
        best_point = (start_i + end_i) // 2
        
        return best_point

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """

        #TO DO:  
        #       1. Get LiDAR message
        #       2. Find closest point to LiDAR
        #       3. Eliminate all points inside 'bubble' (set them to zero) 
        #       4. Fine the max gap
        #       4. Find the best point
        #       5. Publish Drive message
        self.lidar = list(data.ranges)
        NUM_RANGES = len(self.lidar)
        NUM_PER_QUADRANT = NUM_RANGES // 4
        
        min_idx = np.argmin(self.lidar[NUM_PER_QUADRANT:-NUM_PER_QUADRANT]) + NUM_PER_QUADRANT

        ### bubble ###
        bubble_radius = 100
        if(min_idx - bubble_radius > 0):
            for i in range(min_idx - bubble_radius, min_idx + bubble_radius):
                # print(i)
                self.lidar[i] = 0
        else:
            for i in range(0, min_idx + bubble_radius):
                # print(i)
                self.lidar[i] = 0

        # for i in range(0, len(self.lidar)):
        #     if self.lidar[i] > 3:
        #         self.lidar[i] = 0

        # for i in range(min_idx - 200, min_idx + 200):
        #     # print(i)
        #     self.lidar[i] = 0

        # print(min_idx, self.lidar[(min_idx - 10):(min_idx + 10)] )
        # print(self.lidar[NUM_PER_QUADRANT:-NUM_PER_QUADRANT])
        # print('start')
   
        free_space_ranges = self.lidar
        [start_i, end_i] = self.find_max_gap(free_space_ranges)
        # print(data.ranges[270])
        # print(data.ranges[810])        
        bp = self.find_best_point(start_i, end_i, self.lidar)
        print(bp)
        angle = (bp) * np.pi / 540 - np.pi
        #print(angle)
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.stamp = rospy.Time.now()
        self.drive_msg.drive.steering_angle = angle
        self.drive_msg.drive.speed = 2.2
        self.drive_pub.publish(self.drive_msg)


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)