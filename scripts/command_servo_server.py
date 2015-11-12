#!/usr/bin/env python

#the command_servo_server file 

from pipebot.srv import *
import rospy
from bbio import *
from bbio.libraries.Servo import *
import Adafruit_BBIO.ADC as ADC
import utils
import time

FEEDBACKTOL = 2
TIME_INCREMENT = 0.25
TIME_MAX = 2
ADC.setup()

servo1 = Servo(PWM1B)  #servo signal input pin assignment 
feedbackPin = "P9_37" # servo feedback pin assignment 

#the handle_command_servo function controls the servo to execute an angle, reads the servo 
#feedback pin, implements feedback voltage to angle calibration, and output as service response
def handle_command_servo(req):
    # ACCUM_TIME = 0
    # feedback_angle = 1800
    
    required_angle = req.angle #25-155
    commanded_angle = utils.required_to_commanded_angle(required_angle) #130-0
    for i in range(4):
        servo1.write(commanded_angle)
        time.sleep(TIME_INCREMENT)
    # expected_feedback = utils.required_to_expected_feedback(required_angle) #25-155

    # print 'Required Angle', required_angle, 'Commanded Angle', commanded_angle, 'Expected Feedback Angle', expected_feedback

    # while ACCUM_TIME < TIME_MAX and abs(feedback_angle - expected_feedback) > FEEDBACKTOL:
    #     servo1.write(commanded_angle)
    #     voltage_val = ADC.read(feedbackPin)
    #     feedback_angle = utils.voltage_to_feedback(voltage_val) #25-155
    #     time.sleep(TIME_INCREMENT)
    #     ACCUM_TIME += TIME_INCREMENT
    #     print 'Voltage', voltage_val, 'Feedback Angle', feedback_angle, 'Accum Time', ACCUM_TIME

    resp = servoSrvResponse()
    resp.feedback = required_angle
    return resp

#command_servo_server function initializes the command_servo_server node and starts command_servo
#service on this node
def command_servo_server():
    rospy.init_node('command_servo_server')
    s = rospy.Service('command_servo', servoSrv, handle_command_servo)
    print "Ready to Command Servo"
    rospy.spin()

if __name__ == "__main__":
    command_servo_server()