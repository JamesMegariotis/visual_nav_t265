#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import rospy
#from sensor_msgs.msg import LaserScan
from dronekit import connect, VehicleMode
import time
import datetime
#import numpy as np
import math

# Define tracking timer class
class TrackTimer:
    def __init__(self):
        self.fctimeus = None
        self.delta = None
        self.debug = False
    def update(self, timeus):
        if self.debug:
            print "Expected timeus: " +str(self.estimate()) +", actual timeus: " +str(timeus)
        # Messages come in or are processed asynchronously, out of order, so a later message may have an earlier timestamp.
        # So ignore any updates that are older than what we already know about
        if timeus > self.fctimeus:
            self.fctimeus = timeus
            self.mytimeus = datetime.datetime.now()
    def actual(self):
        return self.fctimeus
    def estimate(self):
        if self.fctimeus:
            self.delta = datetime.datetime.now() - self.mytimeus
            return int(self.fctimeus + (self.delta.total_seconds() * 1000))
        else:
            return None


def send_vision(t,x,y,z,r,p,yaw):
    msg = vehicle.message_factory.vision_position_estimate_encode(
        int(t),          # time since system boot, not used
        x,          # min distance cm
        y,          # max distance cm
        z,          # current distance, must be int
        r,          # type = laser?
        p,          # onboard id, not used
        yaw     # must be set to MAV_SENSOR_ROTATION_PITCH_270 for mavlink rangefinder, represents downward facing
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


#def callback(msg):
        #scan = np.array (msg.ranges)
        #left = (np.nanmin (scan[1:200:8]))*100
        #center = (np.nanmin (scan[220:420:8]))*100
        #right = np.nanmin ((scan[440:640:8]))*100
        #send_distance_message(left,7)
        #send_distance_message(center,0)
        #send_distance_message(right,1)
        #print "%.2f" %  left ,
        #print "%.2f" %  center ,
        #print "%.2f" %  right



# Connect to the Vehicle.
print("\nConnecting to vehicle")
vehicle = connect('udpin:0.0.0.0:14551', wait_ready=True)
#vehicle = connect('tcp:192.168.2.18:5763', wait_ready=True)

# Create a new tracktimer object to try and keep sync with the flight controller
trackfctime = TrackTimer()
# Setup a listener for all incoming mavlink messages and update our time tracker whenever possible
@vehicle.on_message("*")
def listener_all(self, name, message):
    try:
        trackfctime.update(message.time_usec)
    except:
        pass
    try:
        trackfctime.update(message.time_boot_ms * 1000)
    except:
        pass

#rospy.init_node('scan_values')
#sub = rospy.Subscriber('/camera/scan', LaserScan, callback)
#rospy.spin()

while (True):
    if (trackfctime.estimate()):
        send_vision(trackfctime.estimate(),0,0,0,0,0,0)
    else:
        send_vision(0,0,0,0,0,0,0)
    time.sleep(0.1)


#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

#https://mavlink.io/en/messages/common.html#MAV_SENSOR_ORIENTATION
#Value	Field Name	Description
#0	MAV_SENSOR_ROTATION_NONE	Roll: 0, Pitch: 0, Yaw: 0
#1	MAV_SENSOR_ROTATION_YAW_45	Roll: 0, Pitch: 0, Yaw: 45
#2	MAV_SENSOR_ROTATION_YAW_90	Roll: 0, Pitch: 0, Yaw: 90
#3	MAV_SENSOR_ROTATION_YAW_135	Roll: 0, Pitch: 0, Yaw: 135
#4	MAV_SENSOR_ROTATION_YAW_180	Roll: 0, Pitch: 0, Yaw: 180
#5	MAV_SENSOR_ROTATION_YAW_225	Roll: 0, Pitch: 0, Yaw: 225
#6	MAV_SENSOR_ROTATION_YAW_270	Roll: 0, Pitch: 0, Yaw: 270
#7	MAV_SENSOR_ROTATION_YAW_315	Roll: 0, Pitch: 0, Yaw: 315
