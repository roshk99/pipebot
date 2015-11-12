#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import time
from bbio import *
from bbio.libraries.Servo import *

NECK_SERVO = Servo(PWM1B)

def callback(data):
    #rospy.loginfo("\n");
    rospy.loginfo(rospy.get_name() + ": I heard \n", data)
    #rospy.loginfo("%s\n" % data.data)
    #rospy.loginfo("\n");
    NECK_SERVO(data)


def neck_servo():
    rospy.init_node('neck_servo_subscriber', anonymous=True)
    print "neck_servo_subscriber node starting."
    rospy.Subscriber("neck_servo_command", Float32, callback)
    rospy.spin()


if __name__ == '__main__':
    neck_servo()
