#!/usr/bin/env python
from __future__ import print_function
import sys
from math import pi, atan, sin, cos
import numpy as np
import time
# ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

# TO DO: Tune parameters
# PID CONTROL PARAMS
kp = 2.2
kd = 0.01
ki = 0.001
servo_offset = 0.0
prev_error = 0.0
error = 0.0
integrator = 0.0
prev_time = time.time()

# WALL FOLLOW PARAMS
ANGLE_RANGE = 270  # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.50  # meters
DESIRED_DISTANCE_LEFT = 0.5
VELOCITY = 4  # meters per second
CAR_LENGTH = 0.50  # TfollowLeftraxxas Rally is 20 inches or 0.5 meters


class WallFollow:
    """ Implement Wall Following on the car
    """

    def __init__(self):
        # Topics & Subs, Pubs
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)  #: Subscribe to LIDAR
        self.velocity_sub = rospy.Subscriber("/vesc/odom", Odometry, self.callback_vel)#: Subscribe to VESC

        self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd', AckermannDriveStamped,
                                         queue_size=10)  #: Publish to drive
        # self.start = 0
        # self.timestamp = rospy.Time.now().secs + rospy.Time.now().nsecs / 1000000000
        # self.prev_timestamp = self.timestamp
        # self.time_duration = 0

    def callback_vel(self, data):
        #TO DO: Subscribe Current Velocity
        self.real_velocity = data.twist.twist.linear.x

    def pid_control(self, error, velocity):
        pass
        # TODO: Use kp, ki & kd to implement a PID controller
        #       Example:
        #               drive_msg = AckermannDriveStamped()
        #               drive_msg.header.stamp = rospy.Time.now()
        #               drive_msg.header.frame_id = "laser"
        #               drive_msg.drive.speed = 0
        #               self.drive_pub.publish(drive_msg)

    def lidar_callback(self, data):
        global kp
        global kd
        global ki
        global servo_offset
        global prev_time
        global prev_error
        global error
        global integrator

        stop_condition = 0
        distance_data = data.ranges

        a_idx = 600
        b_idx = 860

        b = distance_data[b_idx]
        a = distance_data[a_idx]

        unit_angle = float(270) / len(distance_data) * pi / float(180)

        theta = unit_angle * (b_idx - a_idx)
        alpha = atan((a * cos(theta) - b) / (a * sin(theta)))

        AB = b * cos(alpha)  # AB = Dt

        # get velocity to get AC length
        drive_msg = AckermannDriveStamped()
        drive_msg.header.frame_id = "laser"
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.speed = VELOCITY

        timestamp = float(time.time())
        time_step = timestamp - prev_time
        prev_time = timestamp

        velocity = self.real_velocity
        AC = velocity * time_step
        CD = AB + AC * sin(alpha)
        print("gap: {}".format(CD))

        # error < 0 means car is on the left to the desired path
        error = CD - DESIRED_DISTANCE_LEFT
        print("error: {}".format(CD))

        # PID control
        integrator += time_step * error
        differentiator = (error - prev_error) / time_step
        PID_input = kp * error + ki * integrator + kd * differentiator

        prev_error = error

        # drive_msg.drive.steering_angle = PID_input

        if (error < 0):
            a_idx = 680
            b_idx = 880
            b = distance_data[b_idx]
            a = distance_data[a_idx]
            theta = unit_angle * (b_idx - a_idx)
            alpha = atan((a * cos(theta) - b) / (a * sin(theta)))
            AB = b * cos(alpha)  # AB = Dt
            AC = velocity * time_step
            CD = AB + AC * sin(alpha)
            # error < 0 means car is on the left to the desired path
            error = CD - DESIRED_DISTANCE_LEFT

            # PID control
            integrator += time_step * error
            differentiator = (error - prev_error) / time_step

            PID_input = kp * error + ki * integrator + kd * differentiator

            prev_error = error

            final_angle = 3 * PID_input

        else:
            final_angle = PID_input

        if final_angle > 0.62:
            drive_msg.drive.steering_angle = 0.62
        elif final_angle < -0.62:
            drive_msg.drive.steering_angle = -0.62
        else:
            drive_msg.drive.steering_angle = final_angle
        # drive_msg.drive.steering_angle = final_angle


        if (drive_msg.drive.steering_angle > 0.34 or drive_msg.drive.steering_angle < -0.34):
            drive_msg.drive.speed = 2
        else:
            drive_msg.drive.speed = VELOCITY

        # ------------------------ Emergency stop --------------------------- #
        stop_range = [distance_data[180], distance_data[360], distance_data[540], distance_data[720],
                      distance_data[900]]

        itr = 0
        for dist in stop_range:
            if dist < 0.7:
                itr += 1
        # print("itr", itr)

        if len(stop_range) == itr:
            stop_condition = 1

        if stop_condition == 1:
            drive_msg.drive.speed = 0
            drive_msg.drive.steering_angle = 0
            stop_signal = 1

        # # ------------------------ Emergency stop2 --------------------------- #
        # stop_range = distance_data[180:900]

        # max_dist = 0
        # for dist in (stop_range):
        #     if max_dist < dist:
        #         max_dist = dist

        # print("max dist", max_dist)

        # if max_dist < 0.6 or stop_condition == 1:
        #     drive_msg.drive.speed = 0
        #     drive_msg.drive.steering_angle = 0
        #     stop_condition = 1
        #     print("Stop Condition")      

        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("WallFollow_node", anonymous=False)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
