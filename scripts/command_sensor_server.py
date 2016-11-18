#!/usr/bin/env python

from pipebot.srv import *
import rospy
import Adafruit_BBIO.ADC as ADC
import math
import utils

ADC.setup()
sensorPin = "P9_39"

def handle_command_sensor(req):
    arr = []
    for i in range(15):
        sensor1 = ADC.read(sensorPin)
        arr.append(sensor1)
    sensor_val = math.fsum(arr)/len(arr)
    distance_val = utils.voltage_to_distance(sensor_val)
    resp = sensorSrvResponse()
    resp.voltage = float(distance_val)
    return resp

def command_sensor_server():
    rospy.init_node('command_sensor_server')
    s = rospy.Service('command_sensor', sensorSrv, handle_command_sensor)
    print "Ready to Command Sensor"
    rospy.spin()

if __name__ == "__main__":
    command_sensor_server()