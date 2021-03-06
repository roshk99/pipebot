#!/usr/bin/env python
import rospy
from bbio import *
from bbio.libraries.Servo import *
from pipebot.msg import *
import Adafruit_BBIO.PWM as PWM
from std_msgs.msg import Int16
import time
import math

MOTION_MODES = ['Normal forward', 'Halt', 'Move backward', 'Turn left forward', 'Turn right forward']
NECK_ANGLE_INCREMENT = 10
HALT_TIME = 2
TURN_DIST = 3
MOVE_SPEED = 2
DIST_TILL_TURN_45 = 21.55 + TURN_DIST
DIST_TILL_TURN_90 = 15.24 + TURN_DIST
NECK_SERVO1 = Servo(PWM1B)
NECK_SERVO2 = Servo(PWM2B)
NORMAL_DUTY_CYCLE = 100
SLOW_DUTY_CYCLE = 100

def motor_publish_command(data):
    pub = rospy.Publisher('motor_command', motorCmd, queue_size = 10)
    msg_to_send = motorCmd()
    msg_to_send.status = data[0]
    msg_to_send.phase = data[1]
    msg_to_send.duty_cycle_percent = data[2]
    pub.publish(msg_to_send)

def neck_servo_publish_command1(angle):
    actual_angle = angle
    actual_angle = max(5, actual_angle)
    actual_angle = min(actual_angle, 175)
    print "Servo1 is commanded to ", angle
    NECK_SERVO1.write(actual_angle)
    
def neck_servo_publish_command2(angle):
    actual_angle = 180 - angle
    actual_angle = max(5, actual_angle)
    actual_angle = min(actual_angle, 175)
    print "Servo2 is commanded to ", angle
    NECK_SERVO2.write(actual_angle)
    
def turning(servo1, servo2, timelist):
    for angle1, angle2, holdtime in zip(servo1, servo2, timelist):
        neck_servo_publish_command1(angle1)
        neck_servo_publish_command2(angle2)
        time.sleep(0.5)
        motor_publish_command([True, True, NORMAL_DUTY_CYCLE])
        time.sleep(holdtime)
        motor_publish_command([False, True, NORMAL_DUTY_CYCLE])
        time.sleep(0.5)
    motor_publish_command([True, True, NORMAL_DUTY_CYCLE])
    time.sleep(2)
    
        
def motion_modes(mode, turn_angle, duty_cycle_plan):
    #print 'Motion Mode:', mode
    #Normal forward
    if mode is 0:
        motor_publish_command([True, True, duty_cycle_plan])
    #Halt
    elif mode is 1:
        motor_publish_command([False, True, 1])
    #Move backwards
    elif mode is 2:
        motor_publish_command([True, False, duty_cycle_plan])
    elif mode is 3 or 4:
        if turn_angle == 45:
            # servo1 = [90, 85, 80, 75, 70, 20, 20, 20, 20, 70, 75, 80, 85, 90]
            # servo2 = [120, 110, 100, 70, 60, 50, 10, 10, 50, 60, 110, 100, 110, 120]
            servo1 = [90, 130, 140, 160, 140, 120, 110, 90, 90]
            servo2 = [90, 90, 90, 90, 110, 120, 140, 110, 90]
            timelist = [3, 2, 6, 8, 3, 2, 4, 3, 7]
            turning(servo1, servo2, timelist)
        elif turn_angle == -45:
            # servo1 = [90, 85, 80, 75, 70, 65, 65, 65, 65, 60, 75, 80, 85, 90]
            # servo2 = [120, 120, 125, 130, 130, 135, 140, 140, 135, 130, 130, 125, 120, 120]
            servo1 = [60, 40, 30, 20, 20, 30, 40, 60, 90]
            servo2 = [90, 90, 90, 90, 70, 60, 40, 70, 90]
            timelist = [3, 2, 6, 8, 5, 2, 4, 3, 7]
            turning(servo1, servo2, timelist)
        elif turn_angle == 90:
            servo1 = [120, 140, 160, 180, 160, 140, 130, 120, 90]
            servo2 = [90, 90, 90, 90, 110, 120, 140, 110, 90]
            timelist = [3, 2, 6, 8, 5, 2, 4, 3, 7]
            turning(servo1, servo2, timelist)
        elif turn_angle == -90:
            servo1 = [90, 70, 20, 20, 30, 40, 60, 70, 90]
            servo2 = [90, 90, 90, 90, 70, 50, 70, 80, 90]
            timelist = [3, 1, 6, 8, 5, 2, 4, 3, 7]
            turning(servo1, servo2, timelist)
        else:
            print 'Unrecognized angle \n'
    else:
        print "ERROR: mode input unvalid. Needs to be an integer from 0 to 4. "
        
def ujoint_turn_algorithm(turn_dir):
    if (turn_dir == 'left'):
        uservo1 = [130, 130, 160, 160, 160, 160, 160]
        uservo2 = [90, 90, 90, 90, 90, 90, 90]
        timelist = [4, 6, 6, 12, 6, 6, 6]
    	turning(uservo1, uservo2, timelist)
    else:
        uservo1 = [50, 50, 20, 20, 20, 20, 20]
        uservo2 = [90, 90, 90, 90, 90, 90, 90]
        timelist = [4, 6, 6, 12, 6, 6, 6]
    	turning(uservo1, uservo2)
    
    
def main(data):
    #Stop the motors
    motion_modes(1, None, None)
    print 'Motors should be stopped'
    time.sleep(HALT_TIME)
    if data.junction_right is True and data.junction_left is False:
        if data.junction == 'YRB':
            print "Junction angle is too sharp to turn. Robot is going to keep straight.\n"
            time.sleep(0.5)
            motion_modes(0, None, NORMAL_DUTY_CYCLE)
        elif data.junction == 'YRF':
            if data.dist_till_turn < 8:
                print 'Too close to turn right now. Continuing straight'
                time.sleep(0.5)
                motion_modes(0, None, NORMAL_DUTY_CYCLE)
            else:
                turning_angle = -45
                motion_modes(0, None, NORMAL_DUTY_CYCLE)
                motion_modes(4, turning_angle, None)
        elif data.junction == 'TR':
            if data.dist_till_turn > DIST_TILL_TURN_90:
                motion_modes(0, None, SLOW_DUTY_CYCLE)
                return
            turning_angle = -90
            motion_modes(4, turning_angle, None)
        else:
            motion_modes(0, None, NORMAL_DUTY_CYCLE)
    elif data.junction_left is True and data.junction_right is False:
        if data.junction == 'YLB':
            print "Junction angle is too sharp to turn. Robot is going to keep straight.\n"
            time.sleep(0.5)
            motion_modes(0, None, NORMAL_DUTY_CYCLE)
        elif data.junction == 'YLF':
            if data.dist_till_turn > DIST_TILL_TURN_45:
                motion_modes(0, None, SLOW_DUTY_CYCLE)
                return
            turning_angle = 45
            motion_modes(3, turning_angle, None)
        elif data.junction == 'TL':
            if data.dist_till_turn > DIST_TILL_TURN_90:
                motion_modes(0, None, SLOW_DUTY_CYCLE)
                return
            turning_angle = 90
            motion_modes(3, turning_angle, None)
        else:
            motion_modes(0, None, NORMAL_DUTY_CYCLE)
    else:
        if data.junction == 'UR':
            ujoint_turn_algorithm('right')
        elif data.junction == 'UL':
            ujoint_turn_algorithm('left')
        else:
            if data.junction == 'YTR':
                print "Robot can only turn left \n"
                if data.dist_till_turn > DIST_TILL_TURN_45:
                    motion_modes(0, None, SLOW_DUTY_CYCLE)
                    return
                turning_angle = 45
                motion_modes(3, turning_angle, None)
            elif data.junction == 'YTL':
                print "Robot can only turn right \n"
                if data.dist_till_turn > DIST_TILL_TURN_45:
                    motion_modes(0, None, SLOW_DUTY_CYCLE)
                    return
                turning_angle = -45
                motion_modes(4, turning_angle, None)
            elif data.junction == 'TLR':
                # choice = raw_input('Please input whether to turn right or left (L/R): ')
                # print '\n'
                choice = 'L'
                if choice == 'L' or 'l':
                    if data.dist_till_turn > DIST_TILL_TURN_90:
                        motion_modes(0, None, SLOW_DUTY_CYCLE)
                        return
                    turning_angle = 90
                    motion_modes(3, turning_angle, None)
                else:
                    if data.dist_till_turn > DIST_TILL_TURN_90:
                        motion_modes(0, None, SLOW_DUTY_CYCLE)
                        return
                    turning_angle = -90
                    motion_modes(4, turning_angle, None)