#!/usr/bin/env python

import rospy
import math
import numpy as np

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32

MINDIST = 0.05299999937415123
MAXDIST = 0.90
rotation = 120*math.pi/180.0
PRINTRAW = False

def main(data):
    anglemin = data.angle_min
    angleincrement = data.angle_increment
    anglemax = data.angle_max
    angles = xfrange(anglemin, anglemax+angleincrement, angleincrement)
    
    distances = data.ranges
    points = []
    xvec = []
    yvec = []
    for angle, distance in zip(angles, distances):
        if distance > MINDIST and distance < MAXDIST:
            x = distance*math.cos(angle+rotation)
            y = distance*math.sin(angle+rotation)
            if y > 0.0:
                xvec.append(x)
                yvec.append(y)
                point = Point32(x=x, y=y, z=0.0)
                points.append(point)
    if PRINTRAW:
        print 'x=', xvec
        print 'y=', yvec
        print ''
        print ''
    points = PointCloud(points=points)
    pub = rospy.Publisher('processedData', PointCloud, queue_size=10)
    pub.publish(points)

    
def xfrange(start, stop, step):
    range_vec = []
    i = 0
    while start + i * step < stop:
        range_vec.append(start + i * step)
        i += 1
    return range_vec