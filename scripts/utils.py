#!/usr/bin/env python

import math
import time


#Global Variables
DELTHETA = 2
MIN_ANGLE = 25
MAX_ANGLE = 155
MID_ANGLE = 90
MID_SKIP = 20
MODE_DESCRIPTION = ["Normal Operation", "Single Data Point", "Full Sweeping Once"]
SLOPESAMPLESIZE = 3
FRONT_HORIZONTAL_ANGLE_LIM = 7
LEFT_VERTICAL_ANGLE_LIM = 70

def angle_range_generate(min_angle, max_angle, deltheta):
    return range(min_angle, max_angle+1, deltheta)
   
def mode_select(mode):
    print MODE_DESCRIPTION[mode]
    print '_________________________________________________________________'
    single_angle = []
    if (mode == 0):
        left_sweep = angle_range_generate(MIN_ANGLE, MID_ANGLE - MID_SKIP/2, DELTHETA)
        right_sweep = angle_range_generate(MID_ANGLE + MID_SKIP/2, MAX_ANGLE, DELTHETA)
        angle_range = left_sweep + right_sweep
    elif (mode == 1):
        single_angle = input('Single angle testing: type a number between 25 and 155 \n')
        angle_range = [single_angle]
    else:
        angle_range = angle_range_generate(MIN_ANGLE, MAX_ANGLE, DELTHETA)
    return angle_range

def required_to_commanded_angle(required_angle):
    return 180 - (required_angle + 25)
    
#the servo voltage to angle feedback conversion function 
def voltage_to_feedback(voltage):
    a = 8.9758
    b = 59.312
    c = -259.201
    d = 227.342
    feedback = a*(voltage**3) + b*(voltage**2) + c*(voltage) + d
    return feedback
    
def voltage_to_distance(voltage):
    a = -0.2113
    b = 0.4242
    c = -0.3121
    d = 0.2657
    e = 0.012
    inv_distance = a*(voltage**4) + b*(voltage**3) + c*(voltage**2) + d*voltage + e
    return (inv_distance**(-1))

def format_to_matlab(data):
    [x,y] = cartesian_generate(data)
    print 'FOR MATLAB:'
    print 'x=', x, '; y=', y, "; plot(x,y,'.', 'MarkerSize', 20);"
    
def data_adjustment(data, original_data):
    [x,y] = cartesian_generate(data)
    #start_slope_test_100 = [i+1 for i in range(len(data)-2) if data[i+2][0] > 100 and data[i][0] < 100 and abs(data[i+1][0] - 100) == min(abs(data[i][0] - 100), abs(data[i+1][0] - 100), abs(data[i+2][0] - 100))]
    #start_slope_test_100 = start_slope_test_100[0]
    start_slope_test_100 = int(math.floor((100 - MIN_ANGLE)/DELTHETA) + 1)
    slope_list = []
    for i in range(SLOPESAMPLESIZE):
        slope_list.append((y[start_slope_test_100+i+1] - y[start_slope_test_100+i+2])/(x[start_slope_test_100+i+1] - x[start_slope_test_100+i+2]))
    left_line_angle_100 = math.atan(math.fsum(slope_list)/SLOPESAMPLESIZE)*180.0/math.pi
    slope_list = []
    for i in range(SLOPESAMPLESIZE):
        slope_list.append((y[start_slope_test_100+i-7] - y[start_slope_test_100+i-6])/(x[start_slope_test_100+i-7] - x[start_slope_test_100+i-6]))
    right_line_angle_90 = math.atan(math.fsum(slope_list)/SLOPESAMPLESIZE)*180.0/math.pi
    #print right_line_angle_90
    if abs(right_line_angle_90) < FRONT_HORIZONTAL_ANGLE_LIM:
        data = original_data
        [x, y] = cartesian_generate(data)
        #implement adjustment
    if left_line_angle_100 > LEFT_VERTICAL_ANGLE_LIM:
        for j in range(len(data)):
            if data[j][0] > 90 and data[j][0] < 117:
                adjust = -8./math.cos(data[j][0]*math.pi/180.) - 20.
                if adjust >= 0:
                    list_convert = list(data[j])
                    list_convert[1] += math.sqrt(adjust)
                    data[j] = list_convert
        [x, y] = cartesian_generate(data)
    return data
        
def cartesian_generate(data):
    x = []
    y = []
    for theta, r in data:
        theta_rad = theta*math.pi/180.0
        x.append(r*math.cos(theta_rad))
        y.append(r*math.sin(theta_rad))
    return [x, y]