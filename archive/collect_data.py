#!/usr/bin/env python

import sys
import rospy
import time
from pipebot.srv import *
from pipebot.msg import *
import math
from scripts import utils

MAXDIST = 40
# XOFFSET = 2.3
ROTATION_ANGLE_L = 10*math.pi/180.0
ROTATION_ANGLE_R = 6*math.pi/180.0

#command_servo function takes actual angle argument 
#the feedback it returns is also in actual angle
def command_servo(angle):
    rospy.wait_for_service('command_servo')
    try:
        command_servo_func = rospy.ServiceProxy('command_servo', servoSrv)
        resp1 = command_servo_func(angle)
        return resp1.feedback  #It's already converted to angle using the voltageToFeedback function in utils
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def command_sensor():
    rospy.wait_for_service('command_sensor')
    try:
        command_sensor_func = rospy.ServiceProxy('command_sensor', sensorSrv)
        resp1 = command_sensor_func(0)
        return resp1.distance
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


#angle_range takes a list of angles of actual angles
def collect_data(angle_range):
    #ready_init = command_to_init(angle_range[0])
    #if ready_init:
    print "Ready to collect data"
    t = time.time()
    data = []

    for angle in angle_range:
        feedback_angle = command_servo(angle)
        distance = command_sensor()
        # if angle < 90:
        #     distance += XOFFSET / math.tan(angle*math.pi/180.0)
        # elif angle > 90:
        #     distance -= XOFFSET * math.tan(angle*math.pi/180.0 - math.pi/2)
            
        data.append((feedback_angle, min(distance, MAXDIST)))
    left_point = [data[-1][1]*math.cos(data[-1][0]*math.pi/180.0), data[-1][1]*math.sin(data[-1][0]*math.pi/180.0)]
    right_point = [data[0][1]*math.cos(data[0][0]*math.pi/180.0), data[0][1]*math.sin(data[0][0]*math.pi/180.0)]
    new_data = []
    for angle, distance in data:
        
        if angle < 90:
            x = distance*math.cos(angle*math.pi/180.0) - right_point[0]
            y = distance*math.sin(angle*math.pi/180.0) - right_point[1]
            xnew = x*math.cos(ROTATION_ANGLE_R) - y*math.sin(ROTATION_ANGLE_R) + right_point[0]
            ynew = y*math.cos(ROTATION_ANGLE_R) + x*math.sin(ROTATION_ANGLE_R) + right_point[1]
            if angle < 46:
                xnew -= 1
        else:
            # if angle > 98 and angle < 117:
            #     adjust = -8./math.cos(angle*math.pi/180.) - 20.
            #     if adjust >= 0:
            #         distance += math.sqrt(adjust)
            x = distance*math.cos(angle*math.pi/180.0) - left_point[0]
            y = distance*math.sin(angle*math.pi/180.0) - left_point[1]
            xnew = x*math.cos(ROTATION_ANGLE_L) - y*math.sin(ROTATION_ANGLE_L) + left_point[0]
            ynew = y*math.cos(ROTATION_ANGLE_L) + x*math.sin(ROTATION_ANGLE_L) + left_point[1]
        
        new_angle = math.atan(ynew/xnew)/math.pi*180.0
        new_distance = math.sqrt(xnew**2 + ynew**2)
        if new_angle < 0:
            new_angle = new_angle + 180.0
        
        new_data.append((new_angle, new_distance))
    elapsed_time = time.time() - t
    print 'Data collection took', elapsed_time, 'secs'
    return [new_data, data]