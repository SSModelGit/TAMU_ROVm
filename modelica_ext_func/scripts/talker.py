#!/usr/bin/env python
# riffed off the talker.py ROS tutorial

import rospy
# from std_msgs.msg import String
from modelica_ext_func.msg import *

def talker():
    pub = rospy.Publisher('numpub', Num, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        output = int(round(rospy.get_time()))
        rospy.loginfo(output)
        pub.publish(output)
        rate.sleep()

if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
