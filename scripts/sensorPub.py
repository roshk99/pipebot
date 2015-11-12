#!/usr/bin/env python

import rospy
import Adafruit_BBIO.ADC as ADC
from pipebot.msg import sensorData
from pipebot.msg import sensorDetection
from collections import deque
import math

ADC.setup() 
sensorDataArray = sensorData()

def sensorPub():
    pubData = rospy.Publisher('sensors', sensorData, queue_size=10)
    rospy.init_node('sensorPub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        arr = []
        for i in range(10):
            sensor1 = ADC.read("P9_39")	# top forward-facing
            arr.append(sensor1)
            rate.sleep()
        sensorDataArray.data = [math.fsum(arr)/len(arr)]
        rospy.loginfo(sensorDataArray)
        pubData.publish(sensorDataArray)
            
            
    
if __name__ == '__main__':
    try:
        sensorPub()
    except rospy.ROSInterruptException:
        pass

    