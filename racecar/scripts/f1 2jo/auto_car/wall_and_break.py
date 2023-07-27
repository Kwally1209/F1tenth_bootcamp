#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry


#TO DO: Tune parameters
#PID CONTROL PARAMS
kp = 5.0
kd = 13.0
ki = 0.01
servo_offset = 0.0
prev_error = 0.0 
integral = 0.0

a_kp = 1.0
a_kd = 0.0
a_ki = 0.9
a_prev_error = 0.0 
a_integral = 0.0



#WALL FOLLOW PARAMS 
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.80 # meters
DESIRED_DISTANCE_LEFT = 0.50
PI = 3.14159265358979
CAR_LENGTH = 0.50 # TfollowLeftraxxas Rally is 20 inches or 0.5 meters

class LowPassFilter(object):
    def __init__(self, cut_off_frequency, ts):
        self.ts = ts
        self.cut_off_frequency = cut_off_frequency
        self.tau = self.get_tau()

        self.prev_data = 0.

    def get_tau(self):
        return 1/ (2 * np.pi * self.cut_off_frequency)
    
    def filter(self, data):
        val = (self.ts * data + self.tau * self.prev_data) / (self.tau + self.ts)
        self.prev_data = val
        return val

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        self.stop_signal = 0
        #Subscriber
        self.velocity_sub = rospy.Subscriber("/vesc/odom", Odometry, self.callback_vel)#: Subscribe to VESC
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)#: Subscribe to LIDAR
        self.emergency_sub = rospy.Subscriber("/vesc/low_level/ackermann_cmd_mux/input/emg", Bool, self.aeb_callback)#: Subscribe to LIDAR
        #self.synchronized_sub = message_filters.ApproximateTimeSynchronizer(
        #    [self.velocity_sub, self.lidar_sub, self.emergency_sub], queue_size=10, slop=0.5)
        #self.synchronized_sub.registerCallback(self.callback)
        
        #Publisher
        self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd', AckermannDriveStamped, queue_size=10)#: Publish to drive
        
        self.prev_error = prev_error
        self.integral = integral

        self.a_prev_error = a_prev_error
        self.a_integral = a_integral

        self.r = rospy.Rate(100) # 100hz
        self.drive_msg = AckermannDriveStamped()
        self.lidar=[]
        self.real_velocity = 0
        self.lpf_accel = LowPassFilter(5., 0.01)
        self.lpf_gap = LowPassFilter(10., 0.01)
        self.lpf_steer = LowPassFilter(5., 0.01)
        self.lpf_vel = LowPassFilter(10., 0.01)

        self.aeb_check = False
        self.ref_velocity = 0
        self.ref_steer=0
        self.ref_angle=0
        self.cumulative_error=[]
        self.cumulative_steer=[]
    def aeb_callback(self, msg):
        # Emergency Brake
        self.aeb_check = msg.data
        #print(type(msg.data))
        if self.aeb_check == True:
            print("AEB!!!")
            self.ref_velocity = 0
        else:
            self.ref_velocity = 1        
        
        self.drive_msg.drive.speed = self.ref_velocity
        
    def callback_vel(self, data):
        #TO DO: Subscribe Current Velocity
        self.real_velocity = data.twist.twist.linear.x
        # self.real_velocity = self.lpf_vel.filter(self.real_velocity)
        #print("real:{}".format(self.real_velocity))
    #def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        
        #if angle < -45 or angle > 225:
        #    return None
        
        #index = int((angle-data.angle_min) / data.angle_increment)
        
        #print(lidar_range)
        #print(index)

        #if index < 0 or index >=len(lidar_range):
        #    return None
        
        #lidar_range = data.ranges[index]
        

        #if math.isnan(lidar_range):
        #    return None

        
        #self.lidar = data.ranges
        #NUM_RANGES = len(self.lidar) # 2155
        #time = 0.1
        #distance_a =self.lidar[(NUM_RANGES) //6]
        #distance_b =self.lidar[(NUM_RANGES) //3]

        #alpha = np.arctan(((distance_a) * np.sqrt(2) / 2 - distance_b) / (distance_a * np.sqrt(2) / 2) )
        #velocity=self.real_velocity
        #AB = distance_b * np.cos(alpha)
        #AC = velocity *time
        #print("AC: {}".format(velocity))
        #CD = AB + AC * np.sin(alpha)
        #print("gap: {}".format(CD))
        #error = CD - DESIRED_DISTANCE_RIGHT
        #print("error: {}".format(error))

        #return CD

    def steer_pid_control(self, error,velocity):
        
        #TODO: Use kp, ki & kd to implement a PID controller
        #       Example:
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.speed=0.8
    
        time = 0.1
        
        P_control = kp * error
        I_control = self.integral + ki * error * time
        D_control = kd * (error - self.prev_error) / time 
        
        u = (P_control + I_control + D_control)*PI/180
        print("Steering: {}".format(u))
        #print("PGAIN: {}".format(P_control))
        #print("IGAIN: {}".format(I_control))
        #print("DGAIN: {}".format(D_control))
        self.cumulative_steer.append(u)

        # drive_msg.drive.steering_angle = final_angle  
        steer = self.lpf_steer.filter(-u)
        if abs(steer) <0.34:
            self.ref_velocity = 1.0
        else:
            self.ref_velocity = 0.5
            steer = np.clip(steer, -0.62, 0.62)
    
        self.ref_angle = steer


  
    def aeb_callback(self, msg): 
        # Emergency Brake
        self.aeb_check = msg.data 
        #print(type(msg.data))
        if self.aeb_check == True:
            print("AEB!!!")
           
        
    def vel_callback(self, data):
        self.real_velocity = data.twist.twist.linear.x

    def lidar_callback(self, data):
        self.lidar =data.ranges
        NUM_RANGES = len(self.lidar) # 2155
        time=0.1
        distance_b = self.lidar[(NUM_RANGES  ) // 6]
        #print("b: {}".format(distance_b))
        distance_a = self.lidar[(NUM_RANGES ) // 3] 
        #print("a: {}".format(distance_a))       
        d1 = self.lidar[(NUM_RANGES * 1) // 4]
        d2 = self.lidar[(NUM_RANGES * 3) // 4]
        d3 = self.lidar[(NUM_RANGES * 1) // 2]
        

        alpha = np.arctan(((distance_a) * np.sqrt(2)/2 - distance_b) / (distance_a * np.sqrt(2)/2) )
        velocity=self.real_velocity
        AB = distance_b * np.cos(alpha)
        print("AB: {}.".format(AB))
        AC = time
        print("AC: {}".format(AC))
        CD = AB + AC * np.sin(alpha)
        print("gap: {}".format(CD))
        error = CD - DESIRED_DISTANCE_RIGHT
        print("error: {}".format(error))
        self.cumulative_error.append(error)
        # if len(self.cumulative_error) > 400:
        #     plt.plot(self.cumulative_error,'r--', label='Steer')
        #     plt.plot(self.cumulative_steer,'b--', label='Error')
        #     #plt.plot(steer,'r--', label='Steer')
        #     #plt.plot(time,error,'bs', label='Error')
        #     plt.legend()
        #     plt.xlabel('step')
        #     plt.ylabel('Value')
        #     plt.show()
        
        self.steer_pid_control(error, self.real_velocity)

        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.stamp = rospy.Time.now()
        self.drive_msg.header.frame_id = "laser"

        # print(len(data.ranges))\

        s = 1.5
        ttc_dfault = 0.18
        a = 250

        center_idx = int(self.ref_angle/np.pi*1077)

        datas = data.ranges[1073-a+center_idx:1073+a+center_idx]

        # print(len(datas))

        r = min(datas)

        ttc = r / s

        print(ttc)        

        #2155  1072 1 1072
        #      0     135     270

        if ttc < ttc_dfault:
            self.stop_signal=self.stop_signal+1
            print("stop") 
        else:
            self.stop_signal=0

        self.drive_msg.drive.speed = 0.8

        if self.stop_signal>2:
            self.drive_msg.drive.speed = 0.0
            print("stop") 
        #self.drive_pub.publish(drive_msg)
        #TO DO:  
        #       1. Get LiDAR message
        #       2. Calculate length to object with angle in lidar scan field of view
        #          and make sure to take care of nans etc.
        #       3. Based on length to object, callback 'pid_control' 
        



        # TTC = 1
        
        # v = max(self.real_velocity, 0.8)
        # print("{}".format(d3 / (v+0.001+0.07)))
        # TTC = d3 / (v+0.001+0.07)
        # if d1 < 0.6 and d2 < 0.6 and d3 < 0.6:
        #     TTC = d3 / (v+0.001+0.07)
            
        # if TTC < 1 or self.stop_signal:
        #     self.stop_signal = True
        #     velocity = 0
        # else:
        #     velocity = 5
        # print("{}".format(velocity))
        #filt_et = self.lpf_gap.filter(error)
        #print("Wall Gap: {}".format(filt_et))
        #self.steer_pid_control(filt_et)

    def publisher(self):
        
        while not rospy.is_shutdown():
            # error = self.ref_velocity - self.real_velocity
            # self.accel_pid_control(error)
            # print("error: {}".format(error))
            # print("accel: {}".format(self.drive_msg.drive.acceleration))
            # ===================================
            # ------ Ackermann msg pub ----------
            # ===================================
            self.drive_msg.header.stamp = rospy.Time.now()
            self.drive_msg.header.frame_id = "laser"
            self.drive_msg.drive.steering_angle = self.ref_angle
            # self.drive_msg.drive.speed = 1.0
            self.drive_pub.publish(self.drive_msg)
            self.r.sleep()



    def callback(self, vel, lidar, emergency):
        self.vel_callback(vel)
        self.lidar_callback(lidar)
        self.aeb_callback(emergency)
        self.publisher()        


def main(args):

    rospy.init_node("WallFollow_node", anonymous=False)
    wf = WallFollow()
    wf.publisher()
    # rospy.spin()

    



if __name__=='__main__':
	main(sys.argv)

