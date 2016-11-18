#!/usr/bin/env python

# This is a Python script that organizes the movement of the robot.
# It calls multiple sub-functions in the process

import rospy
from src import collect_data
from scripts import alg_stage1, utils
import math 

#Global Variables
DELTHETA = 5
MIN_ANGLE = 20
MAX_ANGLE = 160

def main():
	rospy.init_node("conductor")
	
	#Keep repeating while no errors
	while not rospy.is_shutdown():
		#Stage 1: General Sweeping

		#Initialize motors moving straight

		#Get data for a range of angles
		angle_range = utils.angle_range(MIN_ANGLE, MAX_ANGLE, DELTHETA)
		raw_data = collect_data.collect_data(angle_range)
		print 'Data:', raw_data
		if (raw_data):
			for i in range(len(raw_data)):
				print 'Actual Angle:', angle_range[i], 'Feedback Angle:', raw_data[i][0], 'Distance:', raw_data[i][1] 
		else:
			print 'No Raw Data'
		utils.format_to_matlab(raw_data)
		
		# #Send that data to the stage 1 algorithm
		# junction = alg_stage1.alg_stage1(data)
		# if junction[0] and not junction[1]:
		# 	print 'Junction on the left'
		# elif not junction[0] and junction[1]:
		# 	print 'Junction on the right'
		# elif junction[0] and junction[1]:
		# 	print 'Two-Sided Junction'
		# else:
		# 	print 'No Junction Detected'

		# #If a joint is detected, move to Stage 2: Classification
		# if junction[0] or junction[1]:
		# 	#Get data for a range of angles
		# 	data = collectData.collectData(theta1, theta2, deltheta)

		# 	#Confirm that junction exists
		# 	junction = alg_stage1.alg_stage1(data)
		# 	if not junction[0] and not junction[1]
		# 		break

		# 	#Send that data to stage 2 algorithm
		# 	classification = alg_stage2.alg_stage2(data, junction)
		# 	print 'Classification:', classification

			#Act on that classification until turn is executed

		#Otherwise, just repeat
		

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
