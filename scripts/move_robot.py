#!/usr/bin/env python
# Author: Sammy Pfeiffer

import rospy
from airos4_msgs.srv import MoveServo, MoveServoRequest, MoveServoResponse

SERVOS_SRV = "/airos4/servo/move_servo"

def move_servos(head_pan=None, eyelid=None, eyebrow=None):
    """
    Do the service calls to the servos service for the given parameter
    :param head_pan: pose of the servo of horizontal movement
    :param eyelid: pose of the eyelid servo
    :param eyebrow: pose of the eyebrow servo
    :return:
    """
    try:
        rospy.wait_for_service(SERVOS_SRV, timeout=5)
    except rospy.ROSException as e:
        rospy.logerr("Service: " + SERVOS_SRV + " does not appear to be running.\nReal exception msg: " + str(e))
        return
    servos_srv = rospy.ServiceProxy(SERVOS_SRV, MoveServo)

    # Make all the calls of the same type (Absolute = 0, Relative = 1)
    call_types = 0
    # Make all the calls of the same sync type (async = True, sync = False)
    call_async = False

    if head_pan is not None:
        # Call for head_pan
        try:
            req = MoveServoRequest()
            req.position = head_pan
            req.servo = 0 # head servo, pin 0
            req.type = call_types # Absolute
            req.secs = 0 # Time seems to be ignored
            req.async = call_async
            resp = servos_srv(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    if eyelid is not None:
        # Call for eyelid
        try:
            req = MoveServoRequest()
            req.position = eyelid
            req.servo = 1 # eyelid servo, pin 1
            req.type = call_types # Absolute
            req.secs = 0 # Time seems to be ignored
            req.async = call_async
            resp = servos_srv(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    if eyebrow is not None:
        # Call for eyebrow
        try:
            req = MoveServoRequest()
            req.position = eyebrow
            req.servo = 2 # eyebrow servo, pin 2
            req.type = call_types # Absolute
            req.secs = 0 # Time seems to be ignored
            req.async = call_async
            resp = servos_srv(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e