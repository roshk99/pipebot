#!/usr/bin/env python

import rospy
import motion_plan_generator
from pipebot.msg import *

def main():
    rospy.init_node('ClassificationReceiver', anonymous=True)
    rate = rospy.Rate(10)
    rospy.Subscriber('classificationResult', Classification, motion_plan)
    rospy.spin()
	
def motion_plan(data):
	if not data.junction == 'No result' and not data.junction == 'Straight':
		rospy.loginfo("Junction Left: " + str(data.junction_left) + " , Junction Right: " + str(data.junction_right) + " , Classification: " + str(data.junction))
	else:
		rospy.loginfo("Inconclusive")
	motion_plan_generator.main(data)
	
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass