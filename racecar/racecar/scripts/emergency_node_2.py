#!/usr/bin/env python
import numpy as np
import time

# ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

TARGET_ANGLE = 7
TIME_THRESHOLD = 1.0
MAX_DECEL = -5 

class EmergencyChecker:
    
    def __init__(self):
        self.stop_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/emg", Bool, queue_size=1)  #: Publish to drive
        self.pose_sub = rospy.Subscriber("/vesc/odom", Odometry, self.odom_callback)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.velocity = None
        n_rate = rospy.get_param('~rate', default=20)

        rate = rospy.Rate(n_rate)
        while not rospy.is_shutdown():
            rate.sleep()

    def odom_callback(self,odom):
        self.velocity = odom.twist.twist.linear.x
        if self.velocity < 0:
            self.velocity = 0.1

    def lidar_callback(self, lidar):
        if self.velocity is not None:
            lidar_size = len(lidar.ranges)
            lidar_step = int(np.deg2rad(TARGET_ANGLE) / lidar.angle_increment) # the number of step neep to check

            start = (lidar_size//2)-(lidar_step//2)
            end  = start + lidar_step
            target_ranges = lidar.ranges[start:end]
            msg = Bool()
            DISTANCE_THRESHOLD = self.velocity * TIME_THRESHOLD
            # TTC = np.array(target_ranges) / self.velocity
            # DISTANCE_THRESHOLD = np.array(target_ranges) + 0.5 * MAX_DECEL * (TTC**2)
            # print("target_ranges: {}".format(target_ranges))
            # print("vel: {}".format(self.velocity))
            # print("DISTANCE_THRESHOLD: {}".format(DISTANCE_THRESHOLD))
            # print("TTC: {}".format(TTC))
            DISTANCE_THRESHOLD = np.clip(DISTANCE_THRESHOLD, 0.2, 1.0)

            msg.data = np.any(target_ranges <= DISTANCE_THRESHOLD)
            # msg.data = np.any(TTC <= TIME_THRESHOLD)
            # print("target range: {}".format(target_ranges))
            # print("DISTANCE_THRESHOLD: {}".format(DISTANCE_THRESHOLD))
            print("check: {}".format(msg.data))
            # if np.min(target_ranges) <= 0.5:
            #     msg.data = True

            self.stop_pub.publish(msg)

def main():
    rospy.init_node("emergency_checker", anonymous=False)
    sf = EmergencyChecker()

if __name__ == "__main__":
    main()