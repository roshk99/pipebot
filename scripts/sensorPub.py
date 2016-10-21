#!/usr/bin/env python

# This is a Python script that reads the pin volatages that correspond to the 
# 5 IR sensors. The values are then placed into respective buffers and 
# averaged, to reduce noise in the signal and create a more reliable reading. 
# These readings are then sent through a series of threshold checks to
# determine whether the readings match a joint pattern. This detection is 
# then published in the ROS framework.

import rospy
import Adafruit_BBIO.ADC as ADC
from pipebot.msg import sensorData
from pipebot.msg import sensorDetection
from collections import deque

ADC.setup() 
sensorDataArray = sensorData()
detection = sensorDetection()
sensorBuffer1 = deque([])
sensorBuffer2 = deque([])
sensorBuffer3 = deque([])
sensorBuffer4 = deque([])
sensorBuffer5 = deque([])

listOfBuffers = [sensorBuffer1,sensorBuffer2,sensorBuffer3,sensorBuffer4,sensorBuffer5]

def sensorPub():
    pubData = rospy.Publisher('sensors', sensorData, queue_size=10)
    pubDetect = rospy.Publisher('detection', sensorDetection, queue_size=10)
    rospy.init_node('sensorPub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    bufferLength = 5
    stopped = False
    
    # sensor constants (for easier tweaking)
    wallV = 0.26
    tV = 0.12
    diff = 0.05
    yVaf = 0.15
    yVab = 0.22
    yVbf = 0.155
    yVbb = 0.125
    yVt = 0.09
    
    rospy.loginfo("Sensor signals initialized.")
    while not rospy.is_shutdown():
    	# read all sensors from corresponding pins on BeagleBone Black
        sensor1 = ADC.read("P9_37")	# top forward-facing
        sensor2 = ADC.read("P9_33")	# top left (facing BBB side w/ led's)
        sensor3 = ADC.read("P9_39")	# top right (facing BBB side w/ led's)
        sensor4 = ADC.read("P9_35")	# bottom left (facing BBB side w/ led's)
        sensor5 = ADC.read("P9_40")	# bottom right (facing BBB side w/ led's)
        # place raw sensor data into array
    	sensorDataArray.data = [sensor1,sensor2,sensor3,sensor4,sensor5]


        # place individual sensor data into respective buffers
        for i in range(0,5):
            listOfBuffers[i].append(sensorDataArray.data[i])
            #print 'buffer updated'
            if (len(listOfBuffers[i]) > bufferLength):
                listOfBuffers[i].popleft()
            
        # calculate average of buffer
        avgList = map(lambda x: sum(x)/len(x),listOfBuffers)
        senVal1 = avgList[0]
        senVal2 = avgList[1]
        senVal3 = avgList[2]
        senVal4 = avgList[3]
        senVal5 = avgList[4]

        # display sensor values to the terminal (for debugging and tweaking)
        #rospy.loginfo(sensorDataArray.data)

        
        # logic determining whether a joint is detected
        # if a joint is detected, publish appropriate value to detection topic
        ########################################################################
        ### T - JOINT ###
        # T Right Straight (TRS)
        if ((senVal1 < wallV) and (senVal2 > (wallV-0.02)) and (senVal3 < tV) and (senVal4 > (wallV-0.02)) and (senVal5 < tV) and (abs(senVal3 - senVal5) < diff) and (not stopped)):
            detection.data = 1
            rospy.loginfo("TRS Joint detected.")
            pubDetect.publish(detection.data)
            stopped = True

        # T Left Straight (TLS)
        if ((senVal1 < wallV) and (senVal2 < tV) and (senVal3 > (wallV-0.02)) and (senVal4 < tV) and (senVal5 > (wallV-0.02)) and (abs(senVal2 - senVal4) < diff) and (not stopped)):
            detection.data = 2
            rospy.loginfo("TLS Joint detected.")
            pubDetect.publish(detection.data)
            stopped = True

        # T Left Right (TLR)
        if ((senVal1 > (wallV+0.1)) and (senVal2 < (tV+0.2)) and (senVal3 < (tV+0.2)) and (senVal4 < (tV+0.2)) and (senVal5 < (tV+0.2)) and (abs(senVal3 - senVal5) < (2*diff)) and (abs(senVal2 - senVal4) < (2*diff)) and (not stopped)):
            detection.data = 3
            rospy.loginfo("TLR Joint detected.")
            pubDetect.publish(detection.data)
            stopped = True
            
        ########################################################################
        ### Y - JOINT ###
        # Y Right Ahead (YRA)
        if ((senVal1 < yVt) and (senVal2 > (wallV-0.02)) and (senVal3 < yVaf) and (senVal3 > tV) and (senVal4 > (wallV-0.02)) and (senVal5 < yVab) and (senVal5 > tV) and (((sensor5 - (sensor3+0.03)) > 0) and ((senVal5 - (senVal3+0.02)) > 0)) and (not stopped)):
            detection.data = 4
            rospy.loginfo("YRA Joint detected.")
            pubDetect.publish(detection.data)
            stopped = True

        # Y Right Behind (YRB)
        if ((senVal1 < yVt) and (senVal2 > (wallV-0.02)) and (senVal3 < yVbf) and (senVal4 > (wallV-0.02)) and (senVal5 < yVbb) and (senVal5 > tV)  and ((((sensor3+0.03) - sensor5) > 0) and (((senVal3+0.02) - senVal5) > 0)) and (not stopped)):
            detection.data = 5
            rospy.loginfo("YRB Joint detected.")
            pubDetect.publish(detection.data)
            stopped = True


    	# Y Left Ahead (YLA)
        if ((senVal1 < yVt) and (senVal3 > (wallV-0.02)) and (senVal2 < yVaf) and (senVal2 > tV) and (senVal5 > (wallV-0.02)) and (senVal4 < yVab) and (senVal4 > tV) and (((sensor4 - sensor2) > 0) and ((senVal4 - senVal2) > 0)) and (not stopped)):
            detection.data = 6
            rospy.loginfo("YLA Joint detected.")
            pubDetect.publish(detection.data)
            stopped = True

        # Y Left Behind (YLB)
        if ((senVal1 < yVt) and (senVal3 > (wallV-0.02)) and (senVal2 < yVbf) and (senVal5 > (wallV-0.02)) and (senVal4 < yVbb) and (senVal5 > tV)  and (((sensor2 - sensor4) > 0) and ((senVal2 - senVal4) > 0)) and (not stopped)):
            detection.data = 7
            rospy.loginfo("YLB Joint detected.")
            pubDetect.publish(detection.data)
            stopped = True


    	# Y / T Left (YTL)
        if ((senVal1 > (yVt+0.03)) and (senVal1 < wallV) and (senVal2 > (wallV-0.02)) and (senVal3 < yVbf) and (senVal4 > (wallV-0.02)) and (senVal5 < yVbb) and (senVal5 > tV)  and (((sensor3 - sensor5) > 0) and ((senVal3 - senVal5) > 0)) and (not stopped)):
            detection.data = 8
            rospy.loginfo("YTL Joint detected.")
            pubDetect.publish(detection.data)
            stopped = True

    	# Y / T Right (YTR)
        if ((senVal1 > (yVt+0.03)) and (senVal1 < wallV) and (senVal3 > (wallV-0.02)) and (senVal2 < yVbf) and (senVal5 > (wallV-0.02)) and (senVal4 < yVbb) and (senVal5 > tV)  and (((sensor2 - sensor4) > 0) and ((senVal2 - senVal4) > 0)) and (not stopped)):
            detection.data = 9
            rospy.loginfo("YTR Joint detected.")
            pubDetect.publish(detection.data)
            stopped = True
            
        ########################################################################
        ### U - JOINT ###
        # U Right (UR)
        if ((senVal1 > (wallV+0.1)) and (senVal2 > (wallV-0.02)) and (senVal3 < (tV+0.6)) and (senVal4 > (wallV-0.02)) and (senVal5 > (wallV-0.02)) and (not stopped)):
            detection.data = 10
            rospy.loginfo("UR Joint detected.")
            pubDetect.publish(detection.data)
            stopped = True
        
        # U Left (UL)
        if ((senVal1 > (wallV+0.1)) and (senVal2 < (tV+0.6)) and (senVal3 > (wallV-0.02)) and (senVal4 > (wallV-0.02)) and (senVal5 > (wallV-0.02)) and (not stopped)):
            detection.data = 11
            rospy.loginfo("UL Joint detected.")
            pubDetect.publish(detection.data)
            stopped = True


        #######################################################################
        ### RESET ### 
        if ((senVal1 < 0.1) and (senVal2 < 0.1) and (senVal3 < 0.1) and \
            (senVal4 < 0.1) and (senVal5 < 0.1) and (stopped)):
            detection.data = 0
            rospy.loginfo("Joint no longer detected")
            pubDetect.publish(detection.data)
            stopped = False

        
        rate.sleep()

if __name__ == '__main__':
    try:
        sensorPub()
    except rospy.ROSInterruptException:
        pass
