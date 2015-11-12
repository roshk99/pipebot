#!/usr/bin/env python

import sys
import rospy
import time
from pipebot.srv import *
from pipebot.msg import *
import math
from scripts import utils

MAXDIST = 30

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
    data = []
    for angle in angle_range:
        feedback_angle = command_servo(angle)
        distance = command_sensor()
        data.append((feedback_angle, min(distance, MAXDIST)))
    return data