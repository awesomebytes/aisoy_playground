#!/usr/bin/env python
# Author: Bence Magyar

import rospy
from airos4_msgs.srv import SetColor, SetColorRequest

SERVICE_NAME = "/airos4/heart/set_color"

def set_color(red, green, blue):
    try:
        rospy.wait_for_service(SERVICE_NAME, timeout=5)
    except rospy.ROSException as e:
        rospy.logerr("Service: " + SERVICE_NAME + " does not appear to be running.\nReal exception msg: " + str(e))
        return
    service = rospy.ServiceProxy(SERVICE_NAME, SetColor)

    try:
        req = SetColorRequest()
        req.red = red
        req.green = green
        req.blue = blue
        req.time = 1.0
        resp = service(req)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e