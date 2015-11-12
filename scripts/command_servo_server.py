#!/usr/bin/env python

from pipebot.srv import *
import rospy
from bbio import *
from bbio.libraries.Servo import *

servo1 = Servo(PWM1A)
feedbackPin = AIN2

def handle_command_servo(req):
    print 'Angle Command:', req.angle
    servo1.write(req.angle)
    feedback_val = analogRead(feedbackPin)
    print 'Feedback Value:', feedback_val
    
    resp = servoSrvResponse()
    resp.feedback = feedback_val
    return resp

def command_servo_server():
    rospy.init_node('command_servo_server')
    s = rospy.Service('command_servo', servoSrv, handle_command_servo)
    print "Ready to Command Servo"
    rospy.spin()

if __name__ == "__main__":
    command_servo_server()