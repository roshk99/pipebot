#!/usr/bin/env python

# This is a Python script that organizes the movement of the robot.
# It calls multiple sub-functions in the process

import rospy
from src import collect_data
from scripts import utils, algorithm
import math 
import time

ADJUSTMENT = False

def main():
	rospy.init_node("conductor")
	MODE = 2
	time.sleep(0.5)
	
	#Keep repeating while no errors
	while not rospy.is_shutdown():
		if MODE > 0:
			print "============================================================="
			MODE = input('input MODE: type a number among 0, 1, or 2. Normal Operation for 0, single data point for 1, and single sweep for 2 \n')
		
		if MODE == 'EXIT':
			return
		
		#Get data for a range of angles
		angle_range = utils.mode_select(MODE)
		
		[data, original_data] = collect_data.collect_data(angle_range)

		if not MODE == 1 and ADJUSTMENT:
			data2 = utils.data_adjustment(data, original_data)
		else:
			data2 = data
		
		if data:
			print 'Data:', data2
			utils.format_to_matlab(data2)
			if MODE == 0 or MODE == 2:
				#junction = algorithm.algorithm(data, True)
				#print 'Junction Decision'
				#print junction
				pass
		else:
			print 'No Raw Data'

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass