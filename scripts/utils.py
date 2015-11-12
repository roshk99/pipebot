#!/usr/bin/env python

import math
import time

#Global Variables
DELTHETA = 3
MIN_ANGLE = 25
MAX_ANGLE = 155
MID_ANGLE = 90
MID_SKIP = 30
MODE_DESCRIPTION = ["Normal Operation", "Single Data Point", "Full Sweeping Once"]

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
    x = []
    y = []
    for theta, r in data:
        theta_rad = theta*math.pi/180.0
        x.append(r*math.cos(theta_rad))
        y.append(r*math.sin(theta_rad))
    print 'FOR MATLAB:'
    print 'x=', x, '; y=', y, "; plot(x,y,'+');"