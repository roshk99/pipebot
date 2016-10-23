#!/usr/bin/env python

# This is a Python script that contains some useful functions for testing,
# such as generating test proximity data for various junction types

import math
import random
import alg_stage1

def generate_T_data(rad, dist_to_junc, deltheta):
	numPoints = int(180/deltheta)
	data = []
	theta1 = math.atan2(dist_to_junc, rad)

	for i in range(numPoints+1):
		theta = math.pi/numPoints*i
		if theta < theta1 or theta > math.pi - theta1:
			data.append((theta, abs(rad/math.cos(theta))))
		else:
			data.append((theta, min(abs(rad/math.cos(theta)), straight_value)))
	return data


def generate_straight_data(rad, dist_to_junc, deltheta):
	numPoints = int(180/deltheta)
	data = []
	for i in range(numPoints+1):
		theta = math.pi/numPoints*i
		data.append((theta, min(abs(rad/math.cos(theta)), straight_value)))
	return data


def generate_tr_data(rad, dist_to_junc, deltheta):
	numPoints = int(180/deltheta)
	data = []
	theta1 = math.atan2(dist_to_junc, rad)
	theta2 = math.atan2(dist_to_junc+rad, rad)
	for i in range(numPoints+1):
		theta = math.pi/numPoints*i
		if theta1 < theta and theta < theta2:
			data.append((theta, abs((dist_to_junc+rad)/math.sin(theta))))
		else:
			data.append((theta, min(abs(rad/math.cos(theta)), straight_value)))
	return data


def generate_tl_data(rad, dist_to_junc, deltheta):
	numPoints = int(180/deltheta)
	data = []
	theta1 = math.pi - math.atan2(dist_to_junc, rad)
	theta2 = math.pi - math.atan2(dist_to_junc+rad, rad)
	for i in range(numPoints+1):
		theta = math.pi/numPoints*i
		if theta2 < theta and theta < theta1:
			data.append((theta, abs((dist_to_junc+rad)/math.sin(theta))))
		else:
			data.append((theta, min(abs(rad/math.cos(theta)), straight_value)))
	return data


def generate_yr_data(rad, dist_to_junc, deltheta, alpha):
	numPoints = int(180/deltheta)
	data = []
	theta1 = math.atan2(dist_to_junc, rad)
	theta2 = math.atan2(dist_to_junc+2*rad*math.sqrt(2), rad)
	for i in range(numPoints+1):
		theta = math.pi/numPoints*i
		if theta1 < theta and theta < theta2:
			d1 = rad/math.cos(abs(theta))
			x1 = d1*math.sin(abs(theta)) - dist_to_junc
			x2 = 2*rad*math.sqrt(2) - x1
			phi = math.pi/2+alpha - (math.pi/2-abs(theta))
			d2 = x2*math.sin(math.pi/2+alpha)/math.sin(phi)
			d = d1 + d2
			data.append((abs(theta), d))
		else:
			data.append((theta, min(abs(rad/math.cos(theta)), straight_value)))
	return data


def add_error(data, percent):
	if percent == 0:
		data2 = data
		return data2

	data2 = []
	for i in range(len(data)):
		rand1 = random.randrange(1000-percent, 1000+percent)/1000.0
		rand2 = random.randrange(1000-percent, 1000+percent)/1000.0
		data2.append((data[i][0]*rand1, data[i][1]*rand2))
	return data2

#Parameters
rad = 2.5 #radius of pipe
dist_to_junc = 3 #distance to the junction
deltheta = 5 #increment to generate distance data (degrees)
percent = 30 #the error percentage to apply in the data (percent*10) i.e. 35 is 3.5% error
alpha = 60*math.pi/180 #angle of the junction for a y junction (radians)
straight_value = 35 #distance returned when no obstacle ahead

#Generate Data for Specific Type of Junction
#test_data = generate_T_data(rad, dist_to_junc, deltheta)
#test_data = generate_straight_data(rad, dist_to_junc, deltheta)
#test_data = generate_tr_data(rad, dist_to_junc, deltheta)
#test_data = generate_tl_data(rad, dist_to_junc, deltheta)
test_data = generate_yr_data(rad, dist_to_junc, deltheta, alpha)
test_data2 = add_error(test_data, percent)
#print test_data2

#Generate Data for MATLAB
x = []
y = []
for x1, y1 in test_data2:
	x.append(x1)
	y.append(y1)
#Uncomment these two lines and paste output in MATLAB to see plot of data
#print 'x=', x, ';'
#print 'y=', y, '; polar(x,y);'

alg_stage1.alg_stage1(test_data2, True)



