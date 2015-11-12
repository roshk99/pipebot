#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from pipebot.msg import Classification

def classificationGenerator():
    pub = rospy.Publisher('classificationResult', Classification, queue_size=10)
    rospy.init_node('classificationGenerator', anonymous=True)
    msg_to_send = Classification()
    msg_to_send.junction_left = True
    msg_to_send.junction_right = False
    msg_to_send.junction = 'YLF'
    msg_to_send.dist_till_turn = 13
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(msg_to_send)
        print 'Simulating classification result. Input is', msg_to_send
        rate.sleep()
    
if __name__ == '__main__':
    try:
        classificationGenerator()
    except rospy.ROSInterruptException:
        pass