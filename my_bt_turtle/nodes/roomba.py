#!/usr/bin/env ipython
# -*- coding: cp1252 -*-
# {written with Tab indenting 4 spaces}

'''
Created on Jul 19, 2012

@author: dominik
'''

from bluetooth import *
import thread
from struct import *
from collections import namedtuple
from time import gmtime, strftime, sleep
import math
import serial
import re

MAX_VELOCITY = 500
       
t_power_sensor_data = namedtuple('t_power_sensor_data', ['charging_state', 'voltage', 'current', 'temperature', 'charge', 'capacity'])

class Data( object ):
    
    ROOMBA_PULSES_TO_M = 0.000445558279992234
    
    def __init__(self):
        self._sensor_state_struct = struct.Struct(">12B2hBHhb7HBH5B4h2HB6H2B4hb")
        self._last_encoder_counts = None
    
    def _deserialize(self, buffer):
        try:
            (self.bumps_wheeldrops,
            self.wall,
            self.cliff_left, self.cliff_front_left, self.cliff_front_right, self.cliff_right,
            self.virtual_wall,
            self.motor_overcurrents,
            self.dirt_detector_left, self.dirt_detector_right,
            self.remote_opcode, self.buttons,
            self.distance, self.angle,
            self.charging_state,
            self.voltage, self.current, self.temperature, self.charge, self.capacity,
            self.wall_signal, self.cliff_left_signal, self.cliff_front_left_signal,
            self.cliff_front_right_signal, self.cliff_right_signal,
            self.user_digital_inputs, self.user_analog_input,
            self.charging_sources_available,
            self.oi_mode,
            self.song_number, self.song_playing,
            self.number_of_stream_packets,
            self.requested_velocity, self.requested_radius,
            self.requested_right_velocity, self.requested_left_velocity,
            self.encoder_counts_left, self.encoder_counts_right,
            self.light_bumper,
            self.light_bump_left, self.light_bump_front_left, self.light_bump_center_left,
            self.light_bump_center_right, self.light_bump_front_right, self.light_bump_right,
            self.ir_opcode_left, self.ir_opcode_right,
            self.left_motor_current, self.right_motor_current,
            self.main_brish_current, self.side_brush_current,
            self.statis, ) = self._sensor_state_struct.unpack(buffer[0:80])
        except struct.error, e:
            raise roslib.message.DeserializationError(e)

        self.wall = bool(self.wall)
        self.cliff_left = bool(self.cliff_left)
        self.cliff_front_left = bool(self.cliff_front_left)
        self.cliff_front_right = bool(self.cliff_front_right)
        self.cliff_right = bool(self.cliff_right)
        self.virtual_wall = bool(self.virtual_wall)
        self.song_playing = bool(self.song_playing)

        # do unit conversions
        #self.header = std_msgs.Header(stamp=rospy.Time.from_seconds(timestamp))
        self.requested_velocity = float(self.requested_velocity) / 1000.
        self.requested_radius = float(self.requested_radius) / 1000.
        self.requested_right_velocity = float(self.requested_right_velocity) / 1000.
        self.requested_left_velocity = float(self.requested_left_velocity) / 1000.
        
#        self.distance /= 1000.
#        if self.angle != 0:
#            print "raw angle=%f" %self.angle
            
#        self.angle = self.angle / 0.258
#        self.angle = (360 * self.angle) / (258 * math.pi)

        # The distance and angle calculation sent by the robot seems to
        # be really bad. Re-calculate the values using the raw enconder
        # counts.
        if self._last_encoder_counts:
            count_delta_left = self._normalize_encoder_count(
              self.encoder_counts_left - self._last_encoder_counts[0], 0xffff)
            count_delta_right = self._normalize_encoder_count(
              self.encoder_counts_right - self._last_encoder_counts[1], 0xffff)
            distance_left = count_delta_left * self.ROOMBA_PULSES_TO_M
            distance_right = count_delta_right * self.ROOMBA_PULSES_TO_M
            self.distance = (distance_left + distance_right) / 2.0
            self.angle = (distance_right - distance_left) / 0.235
        else:
            self.distance = 0
            self.angle = 0
        
        self._last_encoder_counts = (self.encoder_counts_left, self.encoder_counts_right)
        
    def _normalize_encoder_count(self, count_delta, maximal_count):
        if count_delta >= maximal_count / 2:
            return count_delta - maximal_count + 1
        elif count_delta <= -maximal_count / 2:
            return count_delta + maximal_count + 1
        return count_delta

def LOG(text):
#    rospy.loginfo(text)
    pass

class Roomba( object ):
        
    def __init__(self, address, bluetooth):
        self.mode = None
        self.bluetooth = bluetooth
        self.data = Data()
        if not 'connected' in locals() or self.connected == False:
            if self.bluetooth:
                self.connectRoombaBluetooth(address)
            else:
                self.connectRoombaSerial(address)
 
    def connectRoombaBluetooth(self, address):
        LOG('Connect...')
        
        self.socket = BluetoothSocket( RFCOMM )
        self.socket.connect((address, 1)) 
        self.socket.settimeout(5)
        
        self.connected = True
        
        self.startUp()
        
    def connectRoombaSerial(self, port):
        LOG('Connect...')
        
        self.socket = serial.Serial(port, baudrate=115200)
        self.socket.open()
        
        self.connected = True
        
        #self.startUp()
        
    def send(self, data):
        if self.bluetooth:
            return self.socket.send(data)
        else:
            return self.socket.write(data)
    
    def recv(self, length):
        if self.bluetooth:
            return self.socket.recv(length)
        else:
            return self.socket.read(length)
        
    def startUp(self):
        self.isdocking = False
        
        try:
            # get the sensors to check if roomba is powered on
            self.getPowerSensors()
            self.powered = True
        except:
            self.powered = False
            self.powerOn()
            try:
                self.getPowerSensors()
                self.powered = True
            except:
                return;
            
        self.enableControl()
        self.setPassiveMode();
        
    def disconnect(self):
        
        LOG('Disconnect...')
        self.socket.close()
        self.connected = False
    
    def getPowerSensors(self):
        
        send_format = 'BB'
        cmd = pack(send_format, 142, 3)
        self.send(cmd)
        recv_format = '>BhhBhh'
        recv = self.receive(10)
        data = t_power_sensor_data._make(unpack(recv_format, recv))
        
        if data.charging_state in [1,2,3,5]:
            self.docked = True
            self.isdocking = False
        else:
            self.docked = False
            
        return data

    def getAllSensors(self):
        send_format = 'BB'
        cmd = pack(send_format, 142, 100)
        self.send(cmd)
        recv = self.receive(80)
        self.data._deserialize(recv)
        
        return self.data
          
    def start(self):
        send_format = 'B'
        cmd = pack(send_format, 128)
        self.send(cmd)
    
    def enableControl(self):
        LOG('Enable Control')
        send_format = 'B'
        cmd = pack(send_format, 130)
        self.send(cmd)
        sleep(0.02)
    
    def setSafeMode(self):
        LOG('Set Safe Mode')
        send_format = 'B'
        cmd = pack(send_format, 131)
        self.send(cmd)
        sleep(0.02)
        self.mode = 1
        
    def setFullMode(self):
        LOG('Set Full Mode')
        send_format = 'B'
        cmd = pack(send_format, 132)
        self.send(cmd)
        sleep(0.02)
        self.mode = 2
        
    def setPassiveMode(self):
        LOG('Set Passive Mode')
        self.start()
        sleep(0.02)
        self.mode = 0
    
    def isDocked(self):
        return self.docked
    
    def isDocking(self):
        return self.isdocking;
    
    def dock(self):
        LOG('Dock started')
        self.enableControl()
        self.setSafeMode()
        send_format = 'B'
        cmd = pack(send_format, 143)
        self.send(cmd)
        self.isdocking = True
        
    def undock(self):
        LOG('Undock started')
        self.enableControl()
        self.setSafeMode()
        self.driveBackwards(50)
        sleep(1)
        self.stop()
        
    def move(self, velocity, radius):
        LOG('Drive v=%d, r=%d' %(velocity, radius))
        send_format = '>Bhh'
        cmd = pack(send_format, 137, velocity, radius)
        self.send(cmd)
        
    def stop(self):
        self.move(0,0)
        
    def capSpeed(self, speed):
        speed = min(speed, 100)
        speed = max(speed, 0)
        return speed
        
    def calculateVelocity(self, speed):
        return round(speed / 100.0 * MAX_VELOCITY)
    
    def drive(self, speed, radius):
        abs_speed = math.fabs(speed);
        abs_speed = self.capSpeed(abs_speed)
        velocity = self.calculateVelocity(abs_speed)
        if speed >= 0:
            self.move(velocity, radius)
        else:
            self.move(-velocity, radius)
        
    def driveBackwards(self, speed):
        speed = math.fabs(speed);
        LOG('Drive Back %d%%' %speed)
        speed = self.capSpeed(speed)
        velocity = self.calculateVelocity(speed)
        self.move(-velocity, -32768)
        
    def driveForwards(self, speed):
        speed = math.fabs(speed);
        LOG('Drive Forward %d%%' %speed)
        speed = self.capSpeed(speed)
        velocity = self.calculateVelocity(speed)
        self.move(velocity, -32768)
    
    def rotateRight(self, speed):
        speed = math.fabs(speed);
        LOG('Rotate %d%%' %speed)
        speed = self.capSpeed(speed)
        velocity = self.calculateVelocity(speed)
        self.move(velocity, -1)
    
    def rotateLeft(self, speed):
        speed = math.fabs(speed);
        LOG('Rotate %d%%' %speed)
        speed = self.capSpeed(speed)
        velocity = self.calculateVelocity(speed)
        self.move(velocity, 1)
    
    def powerOn(self):
        if self.bluetooth:
            self.powerOnBluetooth()
        else:
            self.powerOnSerial()
        
    def powerOnBluetooth(self):
        LOG('Power On')
        if not self.sendCmdAndCheckReply('$$$', 'CMD'):
            print "$$$ failed"
            return False
        
        try:
            if not self.sendCmdAndCheckReply('S@,8080\n', 'AOK'):
                print "$$$ failed"
                raise Exception()
            
            if not self.sendCmdAndCheckReply('S&,8000\n', 'AOK'):
                print "S&,8000\n failed"
                raise Exception()
            
            sleep(0.5)
            
            if not self.sendCmdAndCheckReply('S&,8080\n', 'AOK'):
                print "S&,8080\n failed"
                raise Exception()
        finally:
            if not self.sendCmdAndCheckReply('---\n', 'END'):
                return False
            
        return True
    
    def powerOnSerial(self):
        # wake up robot
        self.socket.setRTS (0)
        sleep (0.1)
        self.socket.setRTS (1)
        sleep (2)
        
        # pulse device-detect three times
        for i in range (3):
            self.socket.setRTS (0)
            sleep (0.25)
            self.socket.setRTS (1)
            sleep (0.25)
        
    def sendCmdAndCheckReply(self, cmd, expected):
        self.send(cmd)
        reply = self.recv(128)
        reply = re.sub(r'[\n\r]', '', reply)
        return reply == expected
            
    def receive(self, length):
        result = ''
        while len(result) != length:
            result += self.recv(length)
        return result

    def direct_drive(self, velocity_left, velocity_right):
        # Mask integers to 2 bytes.
        vl = int(velocity_left) & 0xffff
        vr = int(velocity_right) & 0xffff
        
        send_format = '>BHH'
        cmd = pack(send_format, 145, vr, vl)
        self.send(cmd)
    
    def normal_drive(self, velocity, radius):
        """controls Roomba's drive wheels.
    
        NOTE(damonkohler): The following specification applies to both the Roomba
        and the Turtlebot.
    
        The Roomba takes four data bytes, interpreted as two 16-bit signed values
        using two's complement. The first two bytes specify the average velocity
        of the drive wheels in millimeters per second (mm/s), with the high byte
        being sent first. The next two bytes specify the radius in millimeters at
        which Roomba will turn. The longer radii make Roomba drive straighter,
        while the shorter radii make Roomba turn more. The radius is measured from
        the center of the turning circle to the center of Roomba.
    
        A drive command with a positive velocity and a positive radius makes
        Roomba drive forward while turning toward the left. A negative radius
        makes Roomba turn toward the right. Special cases for the radius make
        Roomba turn in place or drive straight, as specified below. A negative
        velocity makes Roomba drive backward.
    
        Also see drive_straight and turn_in_place convenience methods.
    
        """
        # Mask integers to 2 bytes.
        velocity = int(velocity) & 0xffff
        radius = int(radius) & 0xffff
    
        # Pack as shorts to get 2 x 2 byte integers. Unpack as 4 bytes to send.
        # TODO(damonkohler): The 4 unpacked bytes will just be repacked later,
        # that seems dumb to me.
        bytes = struct.unpack('4B', struct.pack('>2H', velocity, radius))
        self.sci.drive(*bytes)

