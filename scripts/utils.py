#!/usr/bin/env python

import math

def angle_range(min_angle, max_angle, deltheta):
    
    return range(min_angle, max_angle+1, deltheta)
    

def actual_angle_list_to_commanded_angle(actual_angles):
    command_angles = []
    command_angles[:] = [180 - (x + 20) for x in actual_angles]
    
    return command_angles
    
#conversion between command angle and actual angle is reversible. This function converts one to the other 
def angle_conversion(input_angle):
    converted_angle = 180 - (input_angle + 20)
    return converted_angle
    
def process_data(raw_data):
    
    processed_data = []
    for feedback, voltage in raw_data:
        print feedback, voltage
    
    return processed_data
    
#the servo voltage to angle feedback conversion function 
def voltage_to_feedback(voltage):
    feedback = 180.0 - (194.69*voltage - 57.306 + 20)
    return int(feedback)
    
def voltage_to_distance(voltage):
    a = -2.282
    b = 3.135
    c = -1.438
    d = 0.624
    e = 0.026
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