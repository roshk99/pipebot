#!/usr/bin/env python

# This is a Python script that organizes the movement of the robot.
# It calls multiple sub-functions in the process

#import rospy
#import Adafruit_BBIO.ADC as ADC
from src import collectData
from scripts import alg_stage1, alg_stage2

#ADC.setup()

#Global Variables
theta1 = 0 #radians
theta2 = math.pi #radians
deltheta = 1*math.pi/180 #radians

def main():

	#Keep repeating while no errors
	while not rospy.is_shutdown():
	for i in range(1):
		#Stage 1: General Sweeping

		#Initialize motors moving straight

		#Get data for a range of angles
		data = collectData.collectData(theta1, theta2, deltheta)

		#Send that data to the stage 1 algorithm
		junction = alg_stage1.alg_stage1(data)
		if junction[0] and not junction[1]:
			print 'Junction on the left'
		elif not junction[0] and junction[1]:
			print 'Junction on the right'
		elif junction[0] and junction[1]:
			print 'Two-Sided Junction'
		else:
			print 'No Junction Detected'

		#If a joint is detected, move to Stage 2: Classification
		if junction[0] or junction[1]:
			#Get data for a range of angles
			data = collectData.collectData(theta1, theta2, deltheta)

			#Confirm that junction exists
			junction = alg_stage1.alg_stage1(data)
			if not junction[0] and not junction[1]
				break

			#Send that data to stage 2 algorithm
			classification = alg_stage2.alg_stage2(data, junction)
			print 'Classification:', classification

			#Act on that classification until turn is executed

		#Otherwise, just repeat

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
