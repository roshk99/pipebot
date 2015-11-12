#!/usr/bin/env python

import sys
import rospy
from pipebot.srv import *

def command_servo_client(angle):
    rospy.wait_for_service('command_servo')
    try:
        command_servo = rospy.ServiceProxy('command_servo', servoSrv)
        print 'angle:', angle
        resp1 = command_servo(angle)
        print 'resp1:', resp1
        return resp1.feedback
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    angle = 90
    print (angle, command_servo_client(angle))