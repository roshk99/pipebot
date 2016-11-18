#!/usr/bin/env python

import rospy
import math
FEEDBACKLOW = 0
FEEDBACKHIGH = 180
VOLTAGELOW = 600
VOLTAGEHIGH = 800

def voltageToFeedback(voltage):
    if (voltage >=VOLTAGELOW and voltage <= VOLTAGEHIGH):
        feedback = (voltage - VOLTAGELOW)/(VOLTAGEHIGH - VOLTAGELOW)*(FEEDBACKHIGH - FEEDBACKLOW) + FEEDBACKLOW
        return feedback
    else:
        return 0