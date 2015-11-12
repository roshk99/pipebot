#!/usr/bin/env python

# This is a Python script that organizes the movement of the robot.
# It calls multiple sub-functions in the process

import rospy
from scripts import algorithm, process_data, motion_plan_generator
import math 
import time
from sensor_msgs.msg import LaserScan, PointCloud
from pipebot.msg import *

CLASSIFICATION1 = '1'
CLASSIFICATION2 = '2'
CLASSIFICATION3 = '3'
INDEX = 0

def main():
	rospy.init_node("conductor")
	rate = rospy.Rate(5) # 10hz
	rospy.Subscriber('scan', LaserScan, process_data.main)
	rospy.Subscriber('processedData', PointCloud, algorithm.main)
	rospy.Subscriber('classificationResult', Classification, motion_plan)
	rospy.spin()
	
def motion_plan(data):
	if not data.junction == 'No result' and not data.junction == 'Straight':
		rospy.loginfo("Junction Left: " + str(data.junction_left) + " , Junction Right: " + str(data.junction_right) + " , Classification: " + str(data.junction) + " , Distance: " + str(data.dist_till_turn))
		global INDEX
		if INDEX == 0:
			global CLASSIFICATION1
			CLASSIFICATION1 = str(data.junction)
			INDEX = 1
		if INDEX == 1:
			global CLASSIFICATION2
			CLASSIFICATION2 = str(data.junction)
			INDEX = 2
		if INDEX == 2:
			global CLASSIFICATION3
			CLASSIFICATION3 = str(data.junction)
			INDEX = 0
		if CLASSIFICATION1 == CLASSIFICATION2 == CLASSIFICATION3 and data.dist_till_turn < 12:
			rospy.loginfo("Junction Detected. Proceeding to Motion Planner")
			motion_plan_generator.main(data)
			CLASSIFICATION1 = '1'
			CLASSIFICATION2 = '2'
			CLASSIFICATION3 = '3'
	else:
		rospy.loginfo("Inconclusive")
	
	
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass