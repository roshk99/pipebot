#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
import time
from bbio import *
from bbio.libraries.Servo import *

NECK_SERVO1 = Servo(PWM1B)
NECK_SERVO2 = Servo(PWM2B)


def callback1(data):
    NECK_SERVO1.write(data.data)
    print data

def callback2(data):
    NECK_SERVO2.write(data.data)
    print data

def neck_servo():
    rospy.init_node('neck_servo_subscriber', anonymous=True)
    print "neck_servo_subscriber node starting."
    rospy.Subscriber("neck_servo_command1", Int16, callback1)
    rospy.Subscriber("neck_servo_command2", Int16, callback2)
    rospy.spin()


if __name__ == '__main__':
    try:
        neck_servo()
    except rospy.ROSInterruptException:
        NECK_SERVO1.detach()
        NECK_SERVO2.detach()
