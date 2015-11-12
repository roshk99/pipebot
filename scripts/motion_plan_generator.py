#!/usr/bin/env python
import rospy
from bbio import *
from pipebot.msg import *
import Adafruit_BBIO.PWM as PWM
from std_msgs.msg import Int16
import time
import math

MOTION_MODES = ['Normal forward', 'Halt', 'Move backward', 'Turn left forward', 'Turn right forward']
NECK_ANGLE_INCREMENT = 10
NORMAL_DUTY_CYCLE = 100
SLOW_DUTY_CYCLE = 40
TURNING_SPEED = 20
TURNING_SEG_TIME = 2
TURNING_TIME_INCREMENT = 1
SERVO_INIT_POS = 90
DIST_TOL = 5
HALT_TIME = 2
TURN_DIST = 3
MOVE_SPEED = 2
DIST_TILL_TURN_45 = 21.55 + TURN_DIST
DIST_TILL_TURN_90 = 15.24 + TURN_DIST

def motor_publish_command(data):
    pub = rospy.Publisher('motor_command', motorCmd, queue_size = 10)
    msg_to_send = motorCmd()
    msg_to_send.status = data[0]
    msg_to_send.phase = data[1]
    msg_to_send.duty_cycle_percent = data[2]
    pub.publish(msg_to_send)

def neck_servo_publish_command1(data):
    pub = rospy.Publisher('neck_servo_command1', Int16, queue_size = 10)
    pub.publish(data)
    
def neck_servo_publish_command2(data):
    pub = rospy.Publisher('neck_servo_command2', Int16, queue_size = 10)
    pub.publish(data)

# def turning_algorithm(turn_dir, turn_angle, servo_current_pos):
#     if (turn_dir is "left"):
#         angle_change_sign = 1
#     elif (turn_dir is "right"):
#         angle_change_sign = -1
#     num_turn_execute = math.floor(turn_angle/NECK_ANGLE_INCREMENT)
#     last_turn_execute = turn_angle - (num_turn_execute - 1) * NECK_ANGLE_INCREMENT
#     for i in range(num_turn_execute - 1):
#         motor_publish_command([True,True, TURNING_SPEED])
#         time.sleep(TURNING_TIME_INCREMENT)
#         motor_publish_command([False, True, TURNING_SPEED])
#         servo_current_pos += NECK_ANGLE_INCREMENT * angle_change_sign
#         neck_servo_publish_command(servo_current_pos)
#     #last execution
#     motor_publish_command([True, True, TURNING_SPEED])
#     time.sleep(TURNING_TIME_INCREMENT)
#     motor_publish_command([False, True, TURNING_SPEED])
#     servo_current_pos += NECK_ANGLE_INCREMENT * angle_change_sign
#     neck_servo_publish_command(servo_current_pos)
#     return servo_current_pos
    
def turning(servo1, servo2):
    time_int = len(servo1)
    motion_modes(0, None, SLOW_DUTY_CYCLE)
    servo1_current_pos = servo1[0]
    servo2_current_pos = servo2[0]
    for t in range(time_int - 1):
        increment_num = TURNING_SEG_TIME/TURNING_TIME_INCREMENT
        servo1_angle_increment = int(math.floor((servo1[t+1] - servo1_current_pos)/increment_num))
        servo2_angle_increment = int(math.floor((servo2[t+1] - servo2_current_pos)/increment_num))
        for i in range(int(math.floor(increment_num - 1))):
            servo1_current_pos += servo1_angle_increment
            servo2_current_pos += servo2_angle_increment
            neck_servo_publish_command1(servo1_current_pos)
            neck_servo_publish_command2(servo2_current_pos)
            time.sleep(TURNING_TIME_INCREMENT)
        neck_servo_publish_command1(servo1[t+1])
        neck_servo_publish_command2(servo2[t+1])
        time.sleep(TURNING_TIME_INCREMENT)
        
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
            servo1 = [90, 90, 95, 100, 100, 105, 110, 110, 105, 100, 100, 95, 90, 90]
            servo2 = [90, 95, 100, 105, 110, 115, 115, 115, 115, 110, 105, 100, 95, 90]
            turning(servo1, servo2)
        elif turn_angle == -45:
            servo1 = []
            servo2 = []
            turning(servo1, servo2)
        elif turn_angle == 90:
            servo1 = []
            servo2 = []
            turning(servo1, servo2)
        elif turn_angle == -90:
            servo1 = []
            servo2 = []
            turning(servo1, servo2)
        else:
            print 'Unrecognized angle \n'
    
    # #Turn left forward
    # elif mode is 3:
    #     reverse_start_angle = turning_algorithm("left", turn_angle, SERVO_INIT_POS)
    #     # Need to implement a safe signal indicating it is safe to go back to neutral position
        
    #     print "turned to the left by", turn_angle
    #     final_angle = turning_algorithm("right", turn_angle, reverse_start_angle)
    #     if final_angle is not SERVO_INIT_POS:
    #         neck_servo_publish_command(SERVO_INIT_POS)
    #         final_angle = SERVO_INIT_POS
    # #Turn right forward
    # elif motion_info.mode is 4:
    #     reverse_start_angle = turning_algorithm("right", turn_angle, SERVO_INIT_POS)
    #     # Need to implement a safe signal indicating it is safe to go back to neutral position
        
    #     print "turned to the left by", motion_info.turn_angle
    #     final_angle = turning_algorithm("left", turn_angle, reverse_start_angle)
    #     if final_angle is not SERVO_INIT_POS:
    #         neck_servo_publish_command(SERVO_INIT_POS)
    #         final_angle = SERVO_INIT_POS
    else:
        print "ERROR: mode input unvalid. Needs to be an integer from 0 to 4. "
        
def ujoint_turn_algorithm(turn_dir):
    # servo_current_pos = SERVO_INIT_POS
    # if (turn_dir is "left"):
    #     angle_change_sign = 1
    # elif (turn_dir is "right"):
    #     angle_change_sign = -1
    # for i in range(9):
    #     motor_publish_command([True,True, TURNING_SPEED])
    #     time.sleep(TURNING_TIME_INCREMENT)
    #     motor_publish_command([False, True, TURNING_SPEED])
    #     servo_current_pos += NECK_ANGLE_INCREMENT * angle_change_sign
    #     neck_servo_publish_command(servo_current_pos)
    # for i in range(9):
    #     motor_publish_command([True,True, TURNING_SPEED])
    #     time.sleep(TURNING_TIME_INCREMENT)
    #     motor_publish_command([False, True, TURNING_SPEED])
    #     servo_current_pos += NECK_ANGLE_INCREMENT * (-angle_change_sign)
    #     neck_servo_publish_command(servo_current_pos)
    uservo1 = []
    uservo2 = []
    if (turn_dir == 'left'):
    	turning(uservo1, uservo2)
    else:
    	turning([180-x for x in uservo1], [180-x for x in uservo2])
    
    
def main(data):
    if data.junction_left is False and data.junction_right is False:
        motion_modes(0, None, NORMAL_DUTY_CYCLE)
    else:
        #stop the motors
        motion_modes(1, None, None)
        print 'should have stopped the motor'
        time.sleep(3)
        time.sleep(HALT_TIME)
        if data.junction_right is True and data.junction_left is False:
            choice = raw_input('Please input whether to turn right or not (Y/N): ')
            print '\n'
            if choice == 'y' or 'Y':
                if data.junction == 'YRB':
                    print "Junction angle is too sharp to turn. Robot is going to keep straight.\n"
                    time.sleep(0.5)
                    motion_modes(0, None, NORMAL_DUTY_CYCLE)
                elif data.junction == 'YRF':
                    if data.dist_till_turn > DIST_TILL_TURN_45:
                        motion_modes(0, None, SLOW_DUTY_CYCLE)
                        return
                    turning_angle = -45
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
            choice = raw_input('Please input whether to turn right or not (Y/N): ')
            print '\n'
            if choice == 'y' or 'Y':
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
                    choice = raw_input('Please input whether to turn right or left (L/R): ')
                    print '\n'
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
