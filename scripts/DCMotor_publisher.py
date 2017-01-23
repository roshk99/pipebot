#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from pipebot.msg import *
import Adafruit_BBIO.PWM as PWM


def motor_publish_command(data):
    pub = rospy.Publisher('motor_command', motorCmd, queue_size = 10)
    msg_to_send = motorCmd()
    rospy.init_node('DCMotor_publisher', anonymous=True)
    print "DCMotor_publisher node starting."
    # rate = rospy.Rate(10) # 10hz
    msg_to_send.status = data[0]
    msg_to_send.change_phase = data[1]
    msg_to_send.phase = data[2]
    msg_to_send.duty_cycle_percent = data[3]
    print data
    pub.publish(msg_to_send)
    # rate.sleep()
    
    
    
if __name__ == '__main__':
    try:
        motor_publish_command([True, True, False, 20])
    except rospy.ROSInterruptException:
        pass