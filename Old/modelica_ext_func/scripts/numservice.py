#!/usr/bin/env python

from modelica_ext_func.srv import *
from modelica_ext_func.msg import *
import rospy

val = 404

def returnServerCall(data):
    global val
    val = data.num

def handle_subscribe_service(req):
    rospy.Subscriber('numpub', Num, returnServerCall)
    print('Returning the value on the numpub topic, %s'%(val))
    # return val
    return val

def numservice():
    s = rospy.Service('numservice', NumServer, handle_subscribe_service)
    print("Ready to return ints.")
    rospy.spin()

if __name__=="__main__":
    rospy.init_node('numservice_server', anonymous=True)
    # rospy.init_node('listener', anonymous=True)
    numservice()