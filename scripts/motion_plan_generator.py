#!/usr/bin/env python
import rospy
from bbio import *
from pipebot.msg import *
import Adafruit_BBIO.PWM as PWM
from std_msgs.msg import Float32
import time
import math

MOTION_MODES = ['Normal forward', 'Halt', 'Move backward', 'Turn left forward', 'Turn right forward']
NECK_ANGLE_INCREMENT = 20
NORMAL_DUTY_CYCLE = 100
NORMAL_DUTY_CYCLE = 40
TURNING_SPEED = 20
TURNING_SEG_TIME = 2
TURNING_TIME_INCREMENT = 0.5
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
    pub = rospy.Publisher('neck_servo_command1', Float32, queue_size = 10)
    rospy.loginfo('Neck servo angle1:', data)
    pub.publish(data)
    
def neck_servo_publish_command2(data):
    pub = rospy.Publisher('neck_servo_command2', Float32, queue_size = 10)
    rospy.loginfo('Neck servo angle2:', data)
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
        servo1_angle_increment = (servo1[t+1] - servo1_current_pos)/increment_num
        servo2_angle_increment = (servo2[t+1] - servo2_current_pos)/increment_num
        for i in range(increment_num - 1):
            servo1_current_pos += 
            neck_servo_publish_command1()
        
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
    else:
        if turn_angle is 45:
            servo1 = []
            servo2 = []
        elif turn_angle is -45:
            servo1 = []
            servo2 = []
        elif turn_angle is 90:
            servo1 = []
            servo2 = []
        elif turn_angle is -90:
            servo1 = []
            servo2 = []
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
    servo_current_pos = SERVO_INIT_POS
    if (turn_dir is "left"):
        angle_change_sign = 1
    elif (turn_dir is "right"):
        angle_change_sign = -1
    for i in range(5):
        motor_publish_command([True,True, TURNING_SPEED])
        time.sleep(TURNING_TIME_INCREMENT)
        motor_publish_command([False, True, TURNING_SPEED])
        servo_current_pos += NECK_ANGLE_INCREMENT * angle_change_sign
        neck_servo_publish_command(servo_current_pos)
    for i in range(5):
        motor_publish_command([True,True, TURNING_SPEED])
        time.sleep(TURNING_TIME_INCREMENT)
        motor_publish_command([False, True, TURNING_SPEED])
        servo_current_pos += NECK_ANGLE_INCREMENT * (-angle_change_sign)
        neck_servo_publish_command(servo_current_pos)
    
    
def main(data):
    if data.junction_left is False and data.junction_right is False:
        motion_modes(0, None, NORMAL_DUTY_CYCLE)
    else:
        #stop the motors
        motion_modes(1, None, None)
        time.sleep(HALT_TIME)
        if data.junction_right is True and data.junction_left is False:
            choice = input('Please input whether to turn right or not (Y/N): ')
            print '\n'
            if choice is 'y' or 'Y':
                if data.junction is 'YRB':
                    print "Junction angle is too sharp to turn. Robot is going to keep straight.\n"
                    time.sleep(0.5)
                    motion_modes(0, None, NORMAL_DUTY_CYCLE)
                elif data.junction is 'YRF':
                    if data.dist_till_turn > DIST_TILL_TURN_45:
                        motion_modes(0, None, SLOW_DUTY_CYCLE)
                        break
                    turning_angle = 45
                    motion_modes(4, turning_angle, None)
                elif data.junction is 'TR':
                    if data.dist_till_turn > DIST_TILL_TURN_90:
                        motion_modes(0, None, SLOW_DUTY_CYCLE)
                        break
                    turning_angle = 90
                    motion_modes(4, turning_angle, None)
            else:
                motion_modes(0, None, NORMAL_DUTY_CYCLE)
        elif data.junction_left is True and data.junction_right is False:
            choice = input('Please input whether to turn right or not (Y/N): ')
            print '\n'
            if choice is 'y' or 'Y':
                if data.junction is 'YLB':
                    print "Junction angle is too sharp to turn. Robot is going to keep straight.\n"
                    time.sleep(0.5)
                    motion_modes(0, None, NORMAL_DUTY_CYCLE)
                elif data.junction is 'YLF':
                    if data.dist_till_turn > DIST_TILL_TURN_45:
                        motion_modes(0, None, SLOW_DUTY_CYCLE)
                        break
                    turning_angle = 45
                    motion_modes(3, turning_angle, None)
                elif data.junction is 'TL':
                    if data.dist_till_turn > DIST_TILL_TURN_90:
                        motion_modes(0, None, SLOW_DUTY_CYCLE)
                        break
                    turning_angle = 90
                    motion_modes(3, turning_angle, None)
            else:
                motion_modes(0, None, NORMAL_DUTY_CYCLE)
        else:
            if data.junction is 'UR':
                ujoint_turn_algorithm('right')
            elif data.junction is 'UL':
                ujoint_turn_algorithm('left')
            else:
                if data.junction is 'YTR':
                    print "Robot can only turn left \n"
                    if data.dist_till_turn > DIST_TILL_TURN_45:
                        motion_modes(0, None, SLOW_DUTY_CYCLE)
                        break
                    turning_angle = 45
                    motion_modes(3, turning_angle, None)
                elif data.junction is 'YTL':
                    print "Robot can only turn right \n"
                    if data.dist_till_turn > DIST_TILL_TURN_45:
                        motion_modes(0, None, SLOW_DUTY_CYCLE)
                        break
                    turning_angle = 45
                    motion_modes(4, turning_angle, None)
                elif data.junction is 'TLR':
                    choice = input('Please input whether to turn right or left (L/R): ')
                    print '\n'
                    turning_angle = 90
                    if choice is 'L' or 'l':
                        if data.dist_till_turn > DIST_TILL_TURN_90:
                            motion_modes(0, None, SLOW_DUTY_CYCLE)
                            break
                        turning_angle = 90
                        motion_modes(3, turning_angle, None)
                    else:
                        if data.dist_till_turn > DIST_TILL_TURN_90:
                            motion_modes(0, None, SLOW_DUTY_CYCLE)
                            break
                        turning_angle = 90
                        motion_modes(4, turning_angle, None)
