#!/usr/bin/env python

import rospy
import numpy as np
import math
from scipy import interpolate, stats
from rdp import rdp
from pipebot.msg import Classification

EPSILON = 0.1
TOL = 0.1
MIN_SEG_LENGTH = 4
TOL2 = 3
TOL3 = 10
ZERO_ANGLE=8
MID_ANGLE = 90
U_ANGLE = 40
Y_ANGLE = 50
Y_ANGLE2 = 65

PRINTBOOL = False
DEBUGBOOL = True

def rad_to_deg(ang):
	return ang*180.0/math.pi

def plot_matlab(x, y, xnew, ynew, segments=False):
	print '_________'
	if segments:
		for segment in segments:
				print 'plot(', list(segment['x']), ',', list(segment['y']), ", 'r', 'LineWidth', 2);hold on;"
	print 'x=', list(x), ';y=', list(y), ';xnew=', xnew, ';ynew=', ynew, ";hold on;plot(xnew, ynew, 'r.', 'MarkerSize', 30); plot(x,y,'k.', 'MarkerSize', 20); axis square;hold off;"
	print '_________'

def algorithm(xvec,yvec, printbool = False, debugbool = False):

	#Get angles and distances from x,y data
	angles = []
	distances = []
	for x, y in zip(xvec, yvec):
		if x == 0.0:
			angle = math.pi/2
		else:
			angle = math.atan2(y,x)
			if angle < 0: angle += 2*math.pi
		angles.append(rad_to_deg(angle))
		distances.append(math.sqrt(x**2 + y**2))

	#Convert to numpy arrays
	angles = np.array(angles)
	x = np.array(xvec)
	y = np.array(yvec)
	distances = np.array(distances)

	#Sort by increasing angle
	sort_idx = np.argsort(angles)
	sort_angles = angles[sort_idx]
	sort_x = x[sort_idx]
	sort_y = y[sort_idx]
	sort_distances = distances[sort_idx]

	#Perform rdp algorithm and get new data
	X = np.array([sort_x, sort_y]).T
	mask = rdp(X, epsilon=0.05, return_mask=True)
	X1 = X[mask]
	xnew = []
	ynew = []
	for point in X1:
		xnew.append(point[0])
		ynew.append(point[1])

	#Get segment break indices
	indxs = np.array(np.where(mask)).flatten()
	if indxs[0] != 0:
		np.insert(indxs, 0, 0)
	if indxs[len(indxs)-1] != len(sort_x):
		indxs = np.append(indxs, len(sort_x))

	#Get the segments (should have 3)
	segments = []
	for ii in range(len(indxs)-1):
		xvec = sort_x[indxs[ii]:indxs[ii+1]]
		yvec = sort_y[indxs[ii]:indxs[ii+1]]
		angle = sort_angles[indxs[ii]:indxs[ii+1]]
		distance = sort_distances[indxs[ii]:indxs[ii+1]]
		if len(distance) > MIN_SEG_LENGTH:
			if abs(distance[0] - distance[1]) > TOL:
				xvec = xvec[1:]
				yvec = yvec[1:]
				angle = angle[1:]
				distance = distance[1:]
			if abs(distance[len(distance)-1] - distance[len(distance)-2]) > TOL:
				xvec = xvec[:len(distance)-1]
				yvec = yvec[:len(distance)-1]
				angle = angle[:len(distance)-1]
				distance = distance[:len(distance)-1]
			segments.append({'x': xvec, 'y': yvec, 'angle': angle, 'distance': distance})

	#Check number of segments
	if len(segments) != 3:
		if printbool:
			print 'Only', len(segments), 'generated'
			plot_matlab(list(sort_x), list(sort_y), xnew, ynew)
	else:
		#Linearize each segment and get slope
		for segment in segments:
			slope, intercept, r_value, p_value, std_err = stats.linregress(segment['x'],segment['y'])
			segment['slope'] = slope

		#Get LEFT and RIGHT values for each segment
		for segment in segments:
			numpoints = len(segment['x'])
			segment['left'] = False
			segment['right'] = False
			angle1 = rad_to_deg(math.atan2(segment['y'][0], segment['x'][0]))
			angle2 = rad_to_deg(math.atan2(segment['y'][numpoints-1], segment['x'][numpoints-1]))
			if angle2 < 90 + TOL2:
				segment['right'] = True
			if angle1 > 90 - TOL2:
				segment['left'] = True
			if angle1 < 90 + TOL2 and angle2 > 90 - TOL2:
				segment['left'] = True
				segment['right'] = True
		
		#Get intermediate angles
		incl_angles = []
		for ii in range(len(segments) - 1):
			m1 = segments[ii]['slope']
			m2 = segments[ii+1]['slope']
			incl_angle = math.atan2(abs(m1-m2), abs(1+m1*m2))
			if incl_angle < 0:
				incl_angle = incl_angle + 2*math.pi
			incl_angles.append(rad_to_deg(incl_angle))
		

		if printbool:
			plot_matlab(x,y,xnew,ynew,segments)
		
		output = rules_engine(segments[0]['slope'], segments[1]['slope'], segments[2]['slope'], incl_angles[0], incl_angles[1])
		if not output:
			return None
		
		distance = get_distance(segments, output)
		#output = (False, True, 'TR')
		#distance = 10
		if debugbool:
			#print segments[0]['left'], segments[0]['right'], segments[1]['left'], segments[1]['right'], segments[2]['left'], segments[2]['right']
			print segments[0]['slope'], segments[1]['slope'], segments[2]['slope']
			print incl_angles
			print distance
		return output, distance

def rules_engine(slope0, slope1, slope2, angle1, angle2):

	results = []
	lefts = 0
	rights = 0
	
	if angle1<10 and abs(angle2-90)>11 and angle2>10 and slope2 < 0:
		results.append('YLF')
		lefts += 1
	if abs(angle1-90)>11 and angle1>10 and angle2<10 and slope0 > 0:
		results.append('YRF')
		rights += 1
	if angle1<10 and abs(angle2-90)<11:
		results.append('TL')
		lefts += 1
	if angle2<10 and abs(angle1-90)<11:
		results.append('TR')
		rights += 1
	if angle1<10 and abs(angle2-90)>11 and angle2>10 and slope2 > 0:
		results.append('YLB')
		lefts += 1
	if abs(angle1-90)>11 and angle1>10 and angle2<10 and slope0 < 0:
		results.append('YRB')
		rights += 1
	if abs(angle1-90)<11 and abs(angle2-90)<11 and abs(slope1)<1:
		results.append('TLR')
		lefts += 1
		rights += 1
	if abs(angle1-90)>11 and abs(angle2-90)>11 and slope1>1:
		results.append('YTL')
		lefts += 1
		rights += 1
	if abs(angle1-90)>11 and abs(angle2-90)>11 and slope1<-1:
		results.append('YTR')
		lefts += 1
		rights += 1
	if abs(angle1-45)<11 and abs(angle2-45)<11 and slope1<1 and slope1>0:
		results.append('UL')
		lefts += 1
		rights += 1
	if abs(angle1-45)<11 and abs(angle2-45)<11 and slope1>-1 and slope1<0:
		results.append('UR')
		lefts += 1
		rights += 1
	if len(results) > 1:
		print 'Results', results
	
	junction_left = False
	junction_right = False
	if len(results) == 0:
		return junction_left, junction_right, 'Straight'
	elif len(results) == 1:
		if lefts > 0:
			junction_left = True
		if rights > 0:
			junction_right = True
		return junction_left, junction_right, results[0]
	else:
		if lefts > 0:
			junction_left = True
		if rights > 0:
			junction_right = True
		return junction_left, junction_right, 'Unknown'

def get_distance(segments, output):
	distance = 0
	if not output[0] and output[1]:
		rightpoint = segments[0]['y'][-1]
		leftpoint = segments[2]['y'][-1]
		distance = rightpoint - leftpoint
	if output[0] and not output[1]:
		rightpoint = segments[0]['y'][0]
		leftpoint = segments[2]['y'][0]
		distance = leftpoint - rightpoint
	if output[0] and output[1]:
		rightpoint = segments[0]['y'][0]
		leftpoint = segments[1]['y'][0]
		distance1 = leftpoint - rightpoint
		rightpoint = segments[1]['y'][-1]
		leftpoint = segments[2]['y'][0]
		distance2 = rightpoint - leftpoint
		if distance1<0 and distance2>0:
			distance = distance2
		if distance2<0 and distance1>0:
			distance = distance1
		if distance1>0 and distance2>0:
			distance = min(distance1, distance2)
		if distance1<0 and distance2<0:
			distance = 0
	return distance*100

def get_x_y(points):
	xvec = []
	yvec = []
	for point in points:
		xvec.append(point.x)
		yvec.append(point.y)
	return xvec, yvec

def main(data):
	if len(data.points) > 0:
		xvec, yvec = get_x_y(data.points)
		result = algorithm(xvec, yvec, PRINTBOOL, DEBUGBOOL)
		if result:
			output = Classification(junction_left=result[0][0], junction_right=result[0][1], junction=result[0][2], dist_till_turn = result[1])
			pub = rospy.Publisher('classificationResult', Classification, queue_size=10)
			pub.publish(output)
		else:
			print "No Result"

def test_main(xvec, yvec):
	result = algorithm(xvec, yvec, PRINTBOOL, DEBUGBOOL)
	print 'result', result

