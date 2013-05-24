#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#Melonee Wise mwise@willowgarage.com

import roslib; roslib.load_manifest('turtle_compass')
import rospy
import math
import copy
import sensor_msgs.msg
import PyKDL
import json
import time

class TurtleCompassCorrection():
    def __init__(self):
        self.count = 0
        self.filter = 1
        # self.cal_offset = 0.0
        self.orientation = 0.0
        self.last_time = 0.0
        self.cal_offset = 0.0
        self.orientation = 0.0
        self.cal_buffer =[]
        self.cal_buffer_length = 1000
        # self.cal_buffer =[]
        # self.cal_buffer_length = 1000
        self.imu_data = sensor_msgs.msg.Imu(header=rospy.Header(frame_id="gyro_link"))
        self.imu_data.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        self.imu_data.angular_velocity_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        self.imu_data.linear_acceleration_covariance = [-1,0,0,0,0,0,0,0,0]
        # self.gyro_measurement_range = rospy.get_param('~gyro_measurement_range', 150.0) 
        # self.gyro_scale_correction = rospy.get_param('~gyro_scale_correction', 1.35)
        self.imu_pub = rospy.Publisher('imu/data', sensor_msgs.msg.Imu)
        # self.imu_pub_raw = rospy.Publisher('imu/raw', sensor_msgs.msg.Imu)

        self.imu_diagnose_sub = rospy.Subscriber('imu_diagnose', sensor_msgs.msg.Imu, self.handleImuDiagnose)

        self.gyro_scale_correction = rospy.get_param('~gyro_scale_correction', 1.35) 

    def handleImuDiagnose(self, data):

        current_time = data.header.stamp
        if self.last_time == 0.0:
            self.last_time = current_time

            self.cal_buffer.append(data.angular_velocity.z)
            if len(self.cal_buffer) > self.cal_buffer_length:
                del self.cal_buffer[:-self.cal_buffer_length]
            self.cal_offset = sum(self.cal_buffer)/len(self.cal_buffer)

            return

        self.count += 1
        if self.count != self.filter:
            return
        else:
            self.count = 0

        dt = (current_time - self.last_time).to_sec()

        self.imu_data.header.stamp = data.header.stamp
        self.imu_data.angular_velocity.z = float((data.angular_velocity.z - self.cal_offset) * 250.0 / 32767.0 * 250.0)
        # self.imu_data.angular_velocity.z = (float(data.angular_velocity.z * 250.0 / 32767.0 * 250.0)-self.cal_offset)/self.cal_offset
        # self.imu_data.angular_velocity.z = -1.0 * self.imu_data.angular_velocity.z
        
        # self.imu_data.orientation = data.orientation

        # past_orientation = self.orientation
        # self.orientation, test = PyKDL.Rotation.Quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w).GetRotAngle()
        # print "Orientation", self.orientation, test
        # self.orientation = self.orientation*(math.pi/180.0)
        
        self.orientation += self.imu_data.angular_velocity.z * dt
        (self.imu_data.orientation.x, self.imu_data.orientation.y, self.imu_data.orientation.z, self.imu_data.orientation.w) = PyKDL.Rotation.RotZ(self.orientation).GetQuaternion()

        self.imu_pub.publish(self.imu_data);
        # rotation = self.orientation - past_orientation
        # (self.imu_data.orientation.x, self.imu_data.orientation.y, self.imu_data.orientation.z, self.imu_data.orientation.w) = PyKDL.Rotation.RotZ(rotation).GetQuaternion()

        self.last_time = current_time

if __name__ == '__main__':
    rospy.init_node('turtle_compass_correction')
    compass_correction = TurtleCompassCorrection()
    try:
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
