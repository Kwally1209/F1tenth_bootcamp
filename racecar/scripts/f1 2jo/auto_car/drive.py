#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import itertools,operator

import concurrent.futures

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import matplotlib.pyplot as plt
import csv

from drivers_ppc import PurePursuitDriver

# choose your drivers here (1-4)
drivers = PurePursuitDriver(lookahead_distance=0.4)


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
        # self.lidar_sub = rospy.Subscriber('/scan',LaserScan, self.ppc_callback)

        self.driver = drivers
        self.pose_sub = rospy.Subscriber('/pf/pose/odom',Odometry , self.ppc_callback)
        self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd',AckermannDriveStamped,queue_size=5)

        self.x_values = []
        self.y_values = []


        self.ref = np.genfromtxt('./map_data_2-centerline.csv', delimiter=',', skip_header = 1)

        self.ref[:,1] = -self.ref[:,1]

        self.ref[:,0] = self.ref[:,0] - 21.916528*2
        self.ref[:,1] = self.ref[:,1] + 10.05*2
        # data[:,0] = data[:,0]-180.8
        # data[:,1] = -1*data[:,1]+154.2

        self.prev_idx = 0


    def ppc_callback(self, data):

        # print(dir(data.pose.pose.position))

        poses = np.array([data.pose.pose.position.x,data.pose.pose.position.y])

        pose_x = poses[0]
        pose_y = poses[1]

        # print(poses)
        pose_theta = data.pose.pose.orientation.w

        speed, steering_angle, lookahead_idx = drivers.pure_pursuit_control(pose_x, pose_y, pose_theta, self.ref)

        # dist = np.abs((self.ref[:,0]-poses[0])**2+(self.ref[:,1]-poses[1])**2-L_f**2)

        # idx = np.argmin(dist)

        # if idx > self.prev_idx and idx-self.prev_idx < 10:
        #     self.prev_idx = idx
        
        # lookahead_idx = self.prev_idx
        
        # # Compute the heading to the lookahead point
        # theta = math.atan2(self.ref[lookahead_idx, 1]-poses[1], self.ref[lookahead_idx, 0]-poses[0])-pose_theta

        # theta = math.atan2(math.sin(theta), math.cos(theta))

        # # Compute the steering angle
        # steering_angle = math.atan2(2.0*math.sin(theta), L_f)

        # # Compute the speed (you may use constant speed simply)
        # speed = 1

        print("idx : ",lookahead_idx)
        print("steer : ", steering_angle)
        
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed

        print(data)


        
        # self.drive_pub.publish(drive_msg)







    #     with open("map_data_2-centerline.csv", "r") as csv_file:
    #         csv_reader = csv.reader(csv_file)
    #         next(csv_reader)  # Skip the header row
    #         for row in csv_reader:
    #             self.x_values.append(float(row[0]))
    #             self.y_values.append(-float(row[1]))

                


    # ################plot##############################

    #     # Create the scatter plot
    #     plt.scatter(self.x_values, self.y_values, c='#1f77b4')
    #     plt.xlabel("X Position")
    #     plt.ylabel("Y Position")
    #     plt.title("Point Mapping")

    #     # Display the plot
    #     new_x = poses[0]
    #     new_y = poses[1]
    #     self.update_plot(new_x, new_y)

    #     self.update_plot(self.ref[lookahead_idx,0],self.ref[lookahead_idx,1])

    #     plt.show(block=False)




    # def update_plot(self, x, y):
    #     self.x_values.append(x)
    #     self.y_values.append(y)
    #     plt.scatter(x, y, color='red')  # Add the new point
    #     plt.pause(0.001)  # Pause to update the plot


        




def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
