#!/usr/bin/env python

import roslib; roslib.load_manifest('myturtle')

import rospy
import tf

import math
from math import sin, cos, pi

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, \
    Vector3

from roomba import *

MAX_WHEEL_SPEED = 500
WHEEL_SEPARATION = 0.235

ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]
ODOM_POSE_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9]

ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]
ODOM_TWIST_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]

class TurtleHandler (object):
    
    def __init__(self):
        rospy.init_node('myturtle')
        self.max_rate = rospy.get_param('~max_rate', 30.0)
        self.address = rospy.get_param('~address')
        self.cmd_vel_timeout = rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.6))
        
        self.roomba = Roomba(self.address, True)
    
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
    
        self.odometryTransformBroadcaster = tf.TransformBroadcaster()
        self.odometryPublisher = rospy.Publisher("odom", Odometry)

        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.receiveVelocityCmd)

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.last_cmd_vel_time = rospy.Time.now()
        
        self.req_cmd_vel = None
        self.last_cmd_vel = 0, 0
        
        rospy.loginfo("Turtle Connected")
    
    def spin(self):
        rate = rospy.Rate(self.max_rate)
        while not rospy.is_shutdown():
    
            self.current_time =  rospy.Time.now()
    
            ## Handle Odometry
    
            self.handleOdometry()
            
            ## Handle Velocity Commands
            
            self.handleVelocity()
            
            self.last_time = self.current_time
            
            rate.sleep()
        
        print "disconnect turtle"
        # shut down roomba (set to passive mode)    
        self.roomba.setPassiveMode();
        self.roomba.disconnect();

    def handleOdometry(self):
        dt = (self.current_time - self.last_time).to_sec()

        self.sensor_data = self.roomba.getAllSensors()
        distance = self.sensor_data.distance
        angle = self.sensor_data.angle
        
#        if angle != 0 or distance != 0:
#            print "a=%.3f, d=%.3f" %(angle, distance),

        delta_x = cos(angle) * distance
        delta_y = -sin(angle) * distance
        
        last_angle = self.th
        self.x += cos(last_angle) * delta_x - sin(last_angle) * delta_y
        self.y += sin(last_angle) * delta_x + cos(last_angle) * delta_y
        self.th += angle
        
#        if angle != 0 or distance != 0:
#            print "x=%.3f, y=%.3f, th=%.3f" %(self.x, self.y, self.th)
      
        odom_quat = (0., 0., sin(self.th/2.), cos(self.th/2.))
        
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = self.current_time
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
        
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(distance/dt, 0, 0), Vector3(0, 0, angle/dt))

        if self.sensor_data.requested_right_velocity == 0 and \
                self.sensor_data.requested_left_velocity == 0 and \
                self.sensor_data.distance == 0:
            odom.pose.covariance = ODOM_POSE_COVARIANCE2
            odom.twist.covariance = ODOM_TWIST_COVARIANCE2
        else:
            odom.pose.covariance = ODOM_POSE_COVARIANCE
            odom.twist.covariance = ODOM_TWIST_COVARIANCE

        self.odometryTransformBroadcaster.sendTransform(
            (self.x, self.y, 0),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )

        self.odometryPublisher.publish(odom)
        
    def handleVelocity(self):
        
        if self.req_cmd_vel is not None:
            
            if self.roomba.mode != 2:
                self.roomba.setFullMode()
            
            req_cmd_vel = self.check_bumpers(self.sensor_data, self.req_cmd_vel);
                            
            self.req_cmd_vel = None
            self.last_cmd_vel_time = self.current_time
        else:
            if self.current_time - self.last_cmd_vel_time > self.cmd_vel_timeout:
                self.last_cmd_vel = 0, 0
                
            req_cmd_vel = self.check_bumpers(self.sensor_data, self.last_cmd_vel)
            
#        self.roomba.move(*req_cmd_vel)
        self.roomba.direct_drive(*req_cmd_vel)
        self.last_cmd_vel = req_cmd_vel
        
#    def move(self, velocity, angle):
##        rospy.loginfo("move v=%d, a=%d" %(velocity, angle))
#        if velocity == 0:
#            if angle > 0:
#                self.roomba.rotateLeft(angle)
#            elif angle < 0:
#                self.roomba.rotateRight(angle)
#            else:
#                self.roomba.stop()
#        else:
#            self.roomba.move(velocity, angle)
    
    
    def receiveVelocityCmd(self, msg):
#        velocity = msg.linear.x * 1000 / MAX_VELOCITY * 100
#        angle = msg.angular.z * 1000 / MAX_VELOCITY * 100
#        self.req_cmd_vel = velocity, angle 
#      
#    def cmd_vel(self, msg):
        # Clamp to min abs yaw velocity, to avoid trying to rotate at low
        # speeds, which doesn't work well.
#        if self.min_abs_yaw_vel is not None and msg.angular.z != 0.0 and abs(msg.angular.z) < self.min_abs_yaw_vel:
#            msg.angular.z = self.min_abs_yaw_vel if msg.angular.z > 0.0 else -self.min_abs_yaw_vel
#        # Limit maximum yaw to avoid saturating the gyro
#        if self.max_abs_yaw_vel is not None and self.max_abs_yaw_vel > 0.0 and msg.angular.z != 0.0 and abs(msg.angular.z) > self.max_abs_yaw_vel: 
#            msg.angular.z = self.max_abs_yaw_vel if msg.angular.z > 0.0 else -self.max_abs_yaw_vel 
        
        # convert twist to direct_drive args
        ts  = msg.linear.x * 1000 # m -> mm
        tw  = msg.angular.z  * (WHEEL_SEPARATION / 2) * 1000 
        # Prevent saturation at max wheel speed when a compound command is sent.
        if ts > 0:
            ts = min(ts,   MAX_WHEEL_SPEED - abs(tw))
        else:
            ts = max(ts, -(MAX_WHEEL_SPEED - abs(tw)))
        self.req_cmd_vel = int(ts - tw), int(ts + tw)

    def check_bumpers(self, s, cmd_vel):
        # Safety: disallow forward motion if bumpers or wheeldrops
        # are activated.
        # TODO: check bumps_wheeldrops flags more thoroughly, and disable
        # all motion (not just forward motion) when wheeldrops are activated
        forward = (cmd_vel[0] + cmd_vel[1]) > 0
        if s.bumps_wheeldrops > 0 and forward:
            return (0,0)
        else:
            return cmd_vel

if __name__ == '__main__':
    turtle = TurtleHandler()
    turtle.spin()
