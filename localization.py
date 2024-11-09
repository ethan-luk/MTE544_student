import sys

from utilities import Logger

from rclpy.time import Time

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters

rawSensors=0
kalmanFilter=1
odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

class localization(Node):
    
    def __init__(self, type, dt, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","stamp"]):

        super().__init__("localizer")

        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.pose=None
        
        if type==rawSensors:
            self.initRawSensors()
        elif type==kalmanFilter:
            self.initKalmanfilter(dt)
        else:
            print("We don't have this type for localization", sys.stderr)
            return  

    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)
        
    def initKalmanfilter(self, dt):
        
        # TODO Part 3: Set up the quantities for the EKF (hint: you will need the functions for the states and measurements)
        
        # x = [x, y, th, w, v, vdot]
        # x= np.array([[self.pose.pose.x, self.pose.pose.y, th, w, v, vdot]])
        x= np.array([[0, 0, 0, 0, 0, 0]])

        Q= 0.5*np.eye(6)

        R= 0.5*np.eye(4)
        
        P=Q # initial covariance
        
        self.kf=kalman_filter(P,Q,R, x, dt)
        
        # TODO Part 3: Use the odometry and IMU data for the EKF
        self.odom_sub=message_filters.Subscriber(self, odom, '/odom') # not sure
        self.imu_sub=message_filters.Subscriber(self, Imu, 'Imu') # not sure
        
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.fusion_callback)
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        
        # TODO Part 3: Use the EKF to perform state estimation
        # Take the measurements
        # your measurements are the linear velocity and angular velocity from odom msg
        # and linear acceleration in x and y from the imu msg
        # the kalman filter should do a proper integration to provide x,y and filter ax,ay
        z= np.array([imu_msg.linear_acceleration.x,
                     imu_msg.linear_acceleration.y,
                     odom_msg.twist.twist.linear.x,
                     odom_msg.twist.twist.angular.z
        ])
        
        # Implement the two steps for estimation
        
        # prediction step?
        self.kf.predict()

        # update step?
        self.kf.update(z) # update based on the current set of measurements
        
        # Get the estimate
        xhat=self.kf.get_states()

        # Update the pose estimate to be returned by getPose
        self.pose=np.array(...)

        # TODO Part 4: log your data
        self.loc_logger.log_values([
            z[0], # imu_ax
            z[1], # imu_ay
            xhat[0], # kf_ax
            xhat[1] * xhat[2], # kf_ay = kf_vx * kf_w
            xhat[1], # kf_vx
            xhat[2], # kf_w
            xhat[3], # kf_x
            xhat[4], # kf_y
            odom_msg.header.stamp, # stamp, not sure whether to use the odom_msg here or imu_msg... maybe neither...?
        ])
      
    def odom_callback(self, pose_msg):
        
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

    # Return the estimated pose
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    spin(LOCALIZER)
