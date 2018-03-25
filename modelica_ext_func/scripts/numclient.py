#!/usr/bin/env python

import sys
import rospy
from modelica_ext_func.srv import *

def numclient():
    rospy.wait_for_service('numservice')
    try:
        numservice = rospy.ServiceProxy('numservice', NumServer)
        resp = numservice()
        return resp.res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
 
def usage():
    return "%s"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 1:
        pass
    else:
        print(usage())
        sys.exit(1)
    print("Requesting val from numpub...")
    print("val of numpub = %s"%numclient())
    # return numclient()