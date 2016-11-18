#!/usr/bin/env python

import sys
import rospy
import time
from pipebot.srv import *
from pipebot.msg import *

FEEDBACKTOL = 3
INITTIMELIMIT = 20
STAYTIMELIMIT = 3
ZEROANGLE = 20

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
        return resp1.voltage
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def command_to_zero():
    servo_feedback_angle = command_servo(ZEROANGLE)
    ready_init = False
    init_time = 0
    while not ready_init:
        time.sleep(1)
        init_time += 1
        servo_feedback_angle = command_servo(ZEROANGLE)
        if abs(servo_feedback_angle - ZEROANGLE) < FEEDBACKTOL:
            ready_init = True
            print "Servo at start position. Feedback is", servo_feedback_angle
        if init_time >= INITTIMELIMIT:
            print "Zeroing servo position timed out. Final Feedback is", servo_feedback_angle
            break
    return ready_init

#angle_range takes a list of angles of actual angles
def collect_data(angle_range):
    ready = False
    stay_time = 0
    ready_init = command_to_zero()
    if ready_init:
        print "Ready to collect data"
        data = []
        for angle in angle_range:
            servo_feedback_angle = command_servo(angle)
            while not ready:
                time.sleep(0.5)
                servo_feedback_angle = command_servo(angle)
                stay_time += 0.5
                if (abs(servo_feedback_angle - angle) < FEEDBACKTOL):
                    ready = True
                if (stay_time >= STAYTIMELIMIT):
                    ready = True
                    servo_feedback_angle = angle #if the feedback angles doesn't give command angle. Then don't use the feedback. this line can be changed 
            if ready:
                distance = command_sensor()
                data.append((servo_feedback_angle, distance))
            stay_time = 0
            ready = False
        return data
        