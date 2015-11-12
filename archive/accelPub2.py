#!/usr/bin/env python

# This is a Python script that reads the pin voltages that correspond to the 
# two accelerometer outputs (x-axis, z-axis). The values are then placed into 
# respective buffers and averaged, to reduce noise in the signal and create a 
# more reliable reading. Both the raw data and buffered data are then 
# published in the ROS framework.
    
import rospy
import Adafruit_BBIO.ADC as ADC
from pipebot.msg import accelData
from collections import deque

ADC.setup()
accelDataArray = accelData()
accelBuffer1 = deque([])
accelBuffer2 = deque([])

listOfBuffers = [accelBuffer1,accelBuffer2]

def accelPub():
    pubAccelData = rospy.Publisher('accel', accelData, queue_size=10)
    
    rospy.init_node('accelPub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    bufferLength = 5
    
    rospy.loginfo("Accelerometer initialized.")
    while not rospy.is_shutdown():
        # read appropriate accelerometer axes for voltage
        accelX = ADC.read("P9_36") # x-axis voltage reading
        accelZ = ADC.read("P9_38") # z-axis voltage reading
        
        # place raw accelerometer data into array
    	accelDataArray.rawData = [accelX,accelZ]

        # place individual accel data into respective buffers
        for i in range(0,2):
            listOfBuffers[i].append(accelDataArray.rawData[i])
            #print 'buffer updated'
            if (len(listOfBuffers[i]) > bufferLength):
                listOfBuffers[i].popleft()
                
        # calculate average of buffer
        avgList = map(lambda x: sum(x)/len(x),listOfBuffers)
        accelValX = avgList[0]
        accelValZ = avgList[1]
        
        # place buffered accel data into array
        accelDataArray.buffData = [accelValX,accelValZ]
        
        # display sensor values to the terminal (for debugging and tweaking)
        # rospy.loginfo(accelDataArray.rawData)
        
        # publish the buffered data for use in motor control node
        pubAccelData.publish(accelDataArray)
        
        rate.sleep()
        
        
if __name__ == '__main__':
    try:
        accelPub()
    except rospy.ROSInterruptException:
        pass