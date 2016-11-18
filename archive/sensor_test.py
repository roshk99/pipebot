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
			data.append((theta, min(abs((rad+dist_to_junc)/math.sin(theta)), straight_value)))
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
			data.append((abs(theta), abs(d)))
		else:
			data.append((theta, min(abs(rad/math.cos(theta)), straight_value)))
	return data


def generate_yl_data(rad, dist_to_junc, deltheta, alpha):
	numPoints = int(180/deltheta)
	data = []
	theta1 = math.pi - math.atan2(dist_to_junc, rad)
	theta2 = math.pi - math.atan2(dist_to_junc+2*rad*math.sqrt(2), rad)
	for i in range(numPoints+1):
		theta = math.pi/numPoints*i
		if theta2 < theta and theta < theta1:
			thetamod = math.pi - theta
			d1 = abs(rad/math.cos(thetamod))
			x1 = d1*math.sin(thetamod) - dist_to_junc
			x2 = 2*rad*math.sqrt(2) - x1
			phi = math.pi/2+alpha - (math.pi/2-thetamod)
			d2 = abs(x2*math.sin(math.pi/2+alpha)/math.sin(phi))
			d = d1 + d2
			data.append((theta, min(straight_value, abs(d))))
		else:
			data.append((theta, min(abs(rad/math.cos(theta)), straight_value)))
	return data

def generate_ylf_data(rad, dist_to_junc, deltheta, alpha):
	numPoints = int(180/deltheta)
	data = []
	dist_to_junc2 = dist_to_junc + 2*rad/math.tan(alpha)
	theta1 = math.atan(dist_to_junc2/rad) - math.pi/2 + alpha
	theta2 = math.pi/2 + alpha - math.atan(dist_to_junc/rad);
	x1 = dist_to_junc2 + 2*rad/math.sin(alpha);
	for i in range(numPoints+1):
	    theta = math.pi/numPoints*i
	    if theta <= theta1:
	        data.append((theta, min(rad/math.cos(math.pi/2 - alpha+theta), straight_value)))
	    elif theta > theta1 and theta <= alpha:
	        data.append((theta, min(x1*math.sin(math.pi/2-alpha)/math.sin(theta), straight_value)))
	    elif theta > alpha and theta < math.pi-theta2:
	        data.append((theta, min(x1*math.sin(alpha)/math.sin(math.pi-theta), straight_value)))
	    else:
	        data.append((theta, min(straight_value, rad*math.cos(alpha)/math.cos(math.pi/2 + alpha - theta))))
	return data

def generate_yrf_data(rad, dist_to_junc, deltheta, alpha):
	numPoints = int(180/deltheta)
	data = []
	dist_to_junc2 = dist_to_junc + 2*rad/math.tan(alpha)
	theta1 = math.atan(dist_to_junc2/rad) - math.pi/2 + alpha
	theta2 = math.pi/2 + alpha - math.atan(dist_to_junc/rad);
	x1 = dist_to_junc2 + 2*rad/math.sin(alpha);
	for i in range(numPoints+1):
	    theta = math.pi/numPoints*i
	    thetamod = math.pi - theta
	    if thetamod <= theta1:
	        data.append((theta, min(rad/math.cos(math.pi/2 - alpha+thetamod), straight_value)))
	    elif thetamod > theta1 and thetamod <= alpha:
	        data.append((theta, min(x1*math.sin(math.pi/2-alpha)/math.sin(thetamod), straight_value)))
	    elif thetamod > alpha and thetamod < math.pi-theta2:
	        data.append((theta, min(x1*math.sin(alpha)/math.sin(math.pi-thetamod), straight_value)))
	    else:
	        data.append((theta, min(straight_value, rad*math.cos(alpha)/math.cos(math.pi/2 + alpha - thetamod))))
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
deltheta = 10 #increment to generate distance data (degrees)
percent = 4 #the error percentage to apply in the data (percent*10) i.e. 35 is 3.5% error
alpha = 45*math.pi/180 #angle of the junction for a y junction (radians)
straight_value = 35 #distance returned when no obstacle ahead
junction_type = 'YR'

#Data Validation
if not 0 < alpha and not alpha < math.pi:
	print "Invalid Alpha Value"

#Generate Junction Data
print "Junction Type:", junction_type
if junction_type == 'T':
	test_data = generate_T_data(rad, dist_to_junc, deltheta)
	print 'Actual Result', [True, True]
elif junction_type == 'Straight':
	test_data = generate_straight_data(rad, dist_to_junc, deltheta)
	print 'Actual Result', [False, False]
elif junction_type == 'TR':
	test_data = generate_tr_data(rad, dist_to_junc, deltheta)
	print 'Actual Result', [False, True]
elif junction_type == 'TL':
	test_data = generate_tl_data(rad, dist_to_junc, deltheta)
	print 'Actual Result', [True, False]
elif junction_type == 'YR':
	test_data = generate_yr_data(rad, dist_to_junc, deltheta, alpha)
	print 'Actual Result', [False, True]
elif junction_type == 'YL':
	test_data = generate_yl_data(rad, dist_to_junc, deltheta, alpha)
	print 'Actual Result', [True, False]
elif junction_type == 'YLF':
	test_data = generate_ylf_data(rad, dist_to_junc, deltheta, alpha)
	print 'Actual Result', [True, True]
elif junction_type == 'YRF':
	test_data = generate_yrf_data(rad, dist_to_junc, deltheta, alpha)
	print 'Actual Result', [True, True]

#Add specified max random error to data
test_data2 = add_error(test_data, percent)

#Generate Data for MATLAB
x = []
y = []
for x1, y1 in test_data2:
	x.append(x1)
	y.append(y1)

junction = alg_stage1.alg_stage1(test_data2, True)
print 'Algorithm Result', junction

#Uncomment these two lines and paste output in MATLAB to see plot of data
print 'x=', x, ';'
print 'y=', y, '; polar(x,y);'