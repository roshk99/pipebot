#!/usr/bin/env python

# This is a Python script that organizes the movement of the robot.
# It calls multiple sub-functions in the process

import rospy
from scripts import algorithm, process_data, motion_plan_generator
import math 
import time
from sensor_msgs.msg import LaserScan, PointCloud
from pipebot.msg import *

BUFFER_LENGTH = 3
	
def main():
	rospy.init_node("conductor")
	rate = rospy.Rate(10) # 10hz
	rospy.Subscriber('scan', LaserScan, process_data.main)
	rospy.Subscriber('processedData', PointCloud, algorithm.main)
	rospy.Subscriber('classificationResult', Classification, motion_plan)
	rospy.spin()
	
def motion_plan(data):
	if not data.junction == 'No result' and not data.junction == 'Straight':
		rospy.loginfo("Junction Left: " + str(data.junction_left) + " , Junction Right: " + str(data.junction_right) + " , Classification: " + str(data.junction) + " , Distance: " + str(data.dist_till_turn))
	else:
		rospy.loginfo("Inconclusive")
	#motion_plan_generator.main(data)
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass