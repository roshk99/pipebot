#!/usr/bin/env python

# This is a Python script that organizes the movement of the robot.
# It calls multiple sub-functions in the process

import rospy
from src import collect_data
from scripts import alg_stage1, utils
import math 

def main():
	rospy.init_node("conductor")
	MODE = 2
	
	#Keep repeating while no errors
	while not rospy.is_shutdown():
		if MODE > 0:
			MODE = input('input MODE: type a number among 0, 1, or 2. Normal Operation for 0, single data point for 1, and single sweep for 2 \n')
		
		if MODE == 'EXIT':
			return
		
		#Get data for a range of angles
		angle_range = utils.mode_select(MODE)
		
		data = collect_data.collect_data(angle_range)
		print 'Data:'
		if data:
			print data
			utils.format_to_matlab(data)
		else:
			print 'No Raw Data'

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass