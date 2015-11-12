#!/usr/bin/env python

import sys
import rospy
from pipebot.srv import *
from pipebot.msg import *

VOLTAGE = 5000

def command_servo(angle):
    rospy.wait_for_service('command_servo')
    try:
        command_servo_func = rospy.ServiceProxy('command_servo', servoSrv)
        print 'At command_servo, angle:', angle
        resp1 = command_servo_func(angle)
        print 'At command_servo, feedback:', resp1.feedback
        return resp1.feedback
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def callback(data):
    print 'At Callback, data:', data.data
    VOLTAGE = data.data
    
    
def sensor_listener():
    print 'Going to start waiting'
    msg = rospy.wait_for_message("sensorData", sensorData)
    print msg    

if __name__ == "__main__":
    #rospy.init_node('senosorListener', anonymous=True)
    
    angle = 45
    feedback = command_servo(angle)
    sensor_listener()
    print 'Angle:', angle, 'Feedback:', feedback, 'Voltage:', VOLTAGE
    