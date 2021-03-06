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

import rospy
import math
import copy
import sensor_msgs.msg
import PyKDL
import json
import time
from SerialGateway import SerialGateway

class TurtleCompass():
    def __init__(self):
        self.cal_offset = 0.0
        self.orientation = 0.0
        self.cal_buffer = []
        self.cal_buffer_length = 1000
        self.imu_data = sensor_msgs.msg.Imu(header=rospy.Header(frame_id="gyro_link"))
        self.imu_data.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        self.imu_data.angular_velocity_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        self.imu_data.linear_acceleration_covariance = [-1,0,0,0,0,0,0,0,0]
        # self.gyro_measurement_range = rospy.get_param('~gyro_measurement_range', 150.0) 
        # self.gyro_scale_correction = rospy.get_param('~gyro_scale_correction', 1.35)
        self.imu_pub = rospy.Publisher('imu/data', sensor_msgs.msg.Imu)
        # self.imu_pub_raw = rospy.Publisher('imu/raw', sensor_msgs.msg.Imu)

        self.imu_diagnose_data = sensor_msgs.msg.Imu(header=rospy.Header(frame_id="gyro_link"))
        self.imu_diagnose_data.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        self.imu_diagnose_data.angular_velocity_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        self.imu_diagnose_data.linear_acceleration_covariance = [-1,0,0,0,0,0,0,0,0]
        self.imu_diagnose_pub = rospy.Publisher('imu_diagnose', sensor_msgs.msg.Imu)

        self.port = rospy.get_param('~compass_port', '/dev/ttyACM0')
        self.baudRate = rospy.get_param('~baudRate', 115200)
        self.serialHandler = SerialGateway(self.port, self.baudRate, self.handleSerial)

    def Start(self):
        self.serialHandler.Start()

    def Stop(self):
        self.serialHandler.Stop()

    def handleSerial(self, line):
        if (len(line) > 0):
            # rospy.loginfo("received: %s" %line)
            try:
                self.data = json.loads(line)
                self.imu_diagnose_data.header.stamp = rospy.Time.now()
                (self.imu_diagnose_data.orientation.x, self.imu_diagnose_data.orientation.y, self.imu_diagnose_data.orientation.z, self.imu_diagnose_data.orientation.w) = PyKDL.Rotation.RotZ(self.data['compass']['heading']*(math.pi/180.0)).GetQuaternion()
                self.imu_diagnose_data.angular_velocity.x = float(self.data['gyro']['x']*(math.pi/180.0))
                self.imu_diagnose_data.angular_velocity.y = float(self.data['gyro']['y']*(math.pi/180.0))
                self.imu_diagnose_data.angular_velocity.z = float(self.data['gyro']['z']*(math.pi/180.0))
                self.imu_diagnose_data.linear_acceleration.x = float(self.data['accelero']['x']*9.81)
                self.imu_diagnose_data.linear_acceleration.y = float(self.data['accelero']['y']*9.81)
                self.imu_diagnose_data.linear_acceleration.z = float(self.data['accelero']['z']*9.81)
                self.imu_diagnose_pub.publish(self.imu_diagnose_data)
            except Exception, e:
                print e
            # self.heading = float(line)
            return

    def reconfigure(self, config, level): 
        # self.gyro_measurement_range = rospy.get_param('~gyro_measurement_range', 150.0) 
        # self.gyro_scale_correction = rospy.get_param('~gyro_scale_correction', 1.35) 
        self.port = rospy.get_param('~compass_port', '/dev/ttyACM0')
        self.baudRate = rospy.get_param('~baudRate', 115200)
        # rospy.loginfo('self.gyro_measurement_range %f' %self.gyro_measurement_range) 
        # rospy.loginfo('self.gyro_scale_correction %f' %self.gyro_scale_correction) 
        rospy.loginfo('compass port %s' %self.port) 
        rospy.loginfo('compass baudRate %d' %self.baudRate)

        self.Start()

    def update_calibration(self, sensor_state):
        # check if we're not moving and update the calibration offset
        # to account for any calibration drift due to temperature
        if sensor_state.requested_right_velocity == 0 and \
               sensor_state.requested_left_velocity == 0 and \
               sensor_state.distance == 0:
        
            self.cal_buffer.append(self.data['gyro']['z'])
            if len(self.cal_buffer) > self.cal_buffer_length:
                del self.cal_buffer[:-self.cal_buffer_length]
            self.cal_offset = sum(self.cal_buffer)/len(self.cal_buffer)
            
    def publish(self, sensor_state, last_time):
        if self.cal_offset == 0:
            self.orient_offset = self.data['compass']['heading']
            return

        current_time = sensor_state.header.stamp
        dt = (current_time - last_time).to_sec()
        # self.orientation = -1.0 * (self.data['compass']['heading'] - self.orient_offset)*(math.pi/180.0)
        self.imu_data.header.stamp =  sensor_state.header.stamp
        self.imu_data.angular_velocity.z = float(self.data['gyro']['z'] - self.cal_offset)*(math.pi/180.0)

        self.orientation += self.imu_data.angular_velocity.z * dt
        #print orientation
        (self.imu_data.orientation.x, self.imu_data.orientation.y, self.imu_data.orientation.z, self.imu_data.orientation.w) = PyKDL.Rotation.RotZ(self.orientation).GetQuaternion()
        self.imu_pub.publish(self.imu_data)
