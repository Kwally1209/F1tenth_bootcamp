#!/usr/bin/env python
from __future__ import print_function
import sys

import numpy as np
import time
import rospy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float64, Bool

class SafetyChecker:

    def __init__(self):
       
        self.emg = 0
        self.mpc_bool = 0
        self.mpc_test = False
        self.gap_test = False
        self.time = 0
        self.trigger = 0
        
        # self.pose_sub = rospy.Subscriber("/vesc/low_level/ackermann_cmd_mux/input/emg", Bool, self.callback_emg)
        self.mpc_sub = rospy.Subscriber("/vesc/low_level/ackermann_cmd_mux/input/mpc", AckermannDriveStamped, self.callback_mpc)
        self.gap_sub = rospy.Subscriber("/vesc/low_level/ackermann_cmd_mux/input/gap", AckermannDriveStamped, self.callback_gap) 
        self.emg_sub = rospy.Subscriber("/vesc/low_level/ackermann_cmd_mux/input/emg", Bool, self.callback_emg)
        
        self.safety_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=1, tcp_nodelay=True)
        raw_input("Enter")
        n_rate = rospy.get_param('~rate', default=20)

        rate = rospy.Rate(n_rate)
        while not rospy.is_shutdown():  
            self.safety = AckermannDriveStamped()
            # print("emg:", self.emg, "mpc:,", self.mpc_test, "gap:,", self.gap_test)
            if self.emg:
                print("Emergency" )
                if self.trigger < n_rate:
                    self.safety.drive.speed = 0
                    self.trigger +=1
                else:
                    self.safety.drive.speed = -0.5

                self.safety.drive.steering_angle = 0    
            elif not self.mpc_bool and self.mpc_test:
                print("MPC" ,"speed : ", self.mpc.drive.speed, "steer: ", self.mpc.drive.steering_angle)
                self.safety = self.mpc  
                self.trigger = 0
            elif self.gap_test:
                print("Reactive", "speed : ", self.gap.drive.speed,"steer: ", self.gap.drive.steering_angle)
                self.safety = self.gap
                self.trigger = 0
            else:
                self.safety.drive.speed = 0
                self.safety.drive.steering_angle = 0 
                self.trigger = 0
        
            self.safety_pub.publish(self.safety)
            self.mpc_test = False
            
            rate.sleep()

        
    def callback_mpc(self, msg):
        self.mpc_bool = msg.drive.jerk
        self.mpc = msg
        self.mpc_test = True

    def callback_gap(self, msg):
        self.gap = msg
        self.gap_test = True
    
    
    def callback_emg(self, msg):
        self.emg = msg.data
        
        

def main():
    rospy.init_node("Safety_Checker", anonymous=False)
    sf = SafetyChecker()


if __name__ == '__main__':
    main()