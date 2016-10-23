#!/usr/bin/env python

# This is a Python script that implements the first stage of classification: junction detection.
# Input:
#	data: list of tuples with the first element in each tuple as an angle and the second a distance
#	radians: boolean that indicates whether the input data is in radians

import math
import random
def alg_stage1(data, radians):
	numPoints = len(data)
	#print 'NumPoints:', numPoints
	
	#Converts the angles to radians if inputted in degrees
	if not radians:
		for i in range(numPoints):
			data[i] = (data[i][0]*math.pi/180.0, data[i][1])
		#print 'Converted to Radians'

	#Use the first and last point to find the horizontal distance
	r_L0 = abs(data[numPoints-1][1]*math.cos(data[numPoints-1][0]))
	r_R0 = abs(data[0][1]*math.cos(data[0][0]))
	print 'r_L0:', r_L0, ', r_R0:', r_R0

	#Calculate the tolerance value based on the horizontal distances and the angle increment
	deltheta = data[1][0] - data[0][0]
	tol = deltheta*(r_L0 + r_R0);
	print 'tol:', tol

	#Sets the initial booleans
	junction = [False, False]

	#For each angle
	for i in range(numPoints):
		#If on the right side of the pipe, calculate the distance value for a straight pipe
		if data[i][0] < math.pi/2:
			compare = r_R0/math.cos(data[i][0])
			flag1 = 'right'
		#If the left side of the pipe, calculate the distance value for a straight pipe
		else:
			compare = r_L0/math.cos(math.pi - data[i][0])
			flag1 = 'left'

		#Compute the absolute difference between the measured value and the expected
		diff = abs(data[i][1] - abs(compare))

		#print 'i:', i, ', theta:', data[i][0]*180/math.pi, flag1, ', diff:', diff

		#If the difference is greater than the tolerance or if the angle is close to the vertical
		#We only care about discrepencies on the sides, not near the vertical
		if diff > tol and abs(data[i][0]-math.pi/2) > 10*math.pi/180:
			#Set that you have found a discrepency on the specific side of the pipe
			if flag1 == 'left':
				junction[0] = True
			else:
				junction[1] = True

	if junction[0] or junction[1]:
		#print 'Junction Detected'
	else:
		#print 'Straight Pipe Ahead'

	return junction
