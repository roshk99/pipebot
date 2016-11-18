#!/usr/bin/env python

#the command_servo_server file 

from pipebot.srv import *
import rospy
from bbio import *
from bbio.libraries.Servo import *
import Adafruit_BBIO.ADC as ADC
import utils

ADC.setup()

servo1 = Servo(PWM1B)  #servo signal input pin assignment 
feedbackPin = "P9_37" # servo feedback pin assignment 

#the handle_command_servo function controls the servo to execute an angle, reads the servo 
#feedback pin, implements feedback voltage to angle calibration, and output as service response
def handle_command_servo(req):
    command_angle = utils.angle_conversion(req.angle)
    servo1.write(command_angle)  
    #below is feedback readout voltage to angle conversion using voltageToFeedback function 
    #from servoFeedbackCalib file
    
    voltage_val = ADC.read(feedbackPin)
    feedback_val = utils.voltage_to_feedback(voltage_val)
    resp = servoSrvResponse()
    resp.feedback = feedback_val
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