#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import itertools, operator

# ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

view_angel = 180  # 0-135
view_idx = int(1077/135*view_angel)

bubble_radius = 50  # in indexes, not meters

class ReactiveFollowGap:
    def __init__(self):
        # Topics & Subscriptions, Publishers
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd', AckermannDriveStamped, queue_size=5)

    def find_max_gap(self, free_space_ranges):
        # mask the bubble
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        # get a slice for each contiguous sequence of non-bubble data
        slices = np.ma.notmasked_contiguous(masked)
        chosen_slice = max(slices, key=lambda x: x.stop - x.start)
        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i):
        return (end_i + start_i) / 2

    def lidar_callback(self, data):
        # Get LiDAR message
        ranges = np.array(data.ranges)[1077-int(view_idx/2):1078+int(view_idx/2)]

        # Find closest point to LiDAR
        closest_range = np.min(ranges)
        closest_i = np.argmin(ranges)

        # Eliminate all points inside 'bubble' (set them to zero) 
        min_idx = max(0, closest_i - bubble_radius)
        max_idx = min(len(ranges)-1, closest_i + bubble_radius)
        ranges[min_idx:max_idx] = 0

        # Find max gap and best point within gap
        start_i, end_i = self.find_max_gap(ranges)
        if start_i is None or end_i is None:
            start_i = 0
            end_i = len(ranges) - 1

        best_i = self.find_best_point(start_i, end_i)

        # Calculate steering angle
        steer_gain = 0.7
        steering_angle = 0.53*(best_i-view_idx/2+1)/280*steer_gain

        # Calculate speed
        if closest_range < 0.2:
            speed = 0.5
        else:
            speed = 1.0

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = ReactiveFollowGap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)