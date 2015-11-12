#!/usr/bin/env python
import rospy
from pipebot.msg import *
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import time

# The DRV8837 needs two PWM signals to control a single motor so:
PWM_pin = "P9_42" 
phase_pin = "P9_41"
REST_TIME = 0.5
FREQ = 2000
DATA_STORE = []
GPIO.setup(phase_pin, GPIO.OUT)

def callback(data):
    #print 'Subscriber', data
    if len(DATA_STORE) is 0:
        prev_status = True
        prev_duty = 1
        prev_phase = True
    else:
        prev_status = DATA_STORE[-1].status
        prev_duty = DATA_STORE[-1].duty_cycle_percent
        prev_phase = DATA_STORE[-1].phase
    if data.status is True:
        if prev_status is True:
            if prev_phase is data.phase:
                if prev_duty is not data.duty_cycle_percent:
                    setDuty(data.duty_cycle_percent)
            else:
                setdirection(data.phase, data.duty_cycle_percent)

        else:
            if data.phase is True:
                GPIO.output(phase_pin, GPIO.LOW)
                PWM.start(PWM_pin, data.duty_cycle_percent, FREQ, 0)
            else:
                GPIO.output(phase_pin, GPIO.HIGH)
                PWM.start(PWM_pin, data.duty_cycle_percent, FREQ, 1)
    else:
        if prev_status is True:
            PWM.stop(PWM_pin)
            PWM.cleanup(PWM_pin)
            GPIO.cleanup(phase_pin)
        else:
            setDuty(0)
    DATA_STORE.append(data)


def setDuty(value):
    ##duty values are valid 0-100
    if value > 100:
        value = 100
    if value < 0:
        value = 0
    PWM.set_duty_cycle(PWM_pin, value)
    
def setdirection(phase, duty):
    if duty > 100:
        duty = 100
    if duty < 0:
        duty = 0
    if phase is False:
        PWM.stop(PWM_pin)
        PWM.cleanup(PWM_pin)
        print "Phase input= 0. Changing direction of motors to backwards. "
        time.sleep(REST_TIME)
        GPIO.output(phase_pin, GPIO.HIGH)
        PWM.start(PWM_pin, duty, FREQ, 1)
        
    if phase is True:
        PWM.stop(PWM_pin)
        PWM.cleanup(PWM_pin)
        print "Phase input= 1. Changing direction of motors to forwards. "
        time.sleep(REST_TIME)
        GPIO.output(phase_pin, GPIO.LOW)
        PWM.start(PWM_pin, duty, FREQ, 0)


def dcmotors():
    rospy.init_node('DCMotor_subscriber', anonymous=True)
    print "DCMotor_subscriber node starting."
    # init the PWM
    ##PWM.start(channel, duty, freq=2000)
    GPIO.output(phase_pin, GPIO.LOW)
    print 'start PWM'
    PWM.start(PWM_pin, 80, FREQ, 0)
    time.sleep(3)
    rospy.Subscriber("motor_command", motorCmd, callback)
    rospy.spin()


if __name__ == '__main__':
    dcmotors()
    if rospy.is_shutdown():
        PWM.stop(PWM_pin)
        PWM.cleanup(PWM_pin)
        GPIO.cleanup(phase_pin)