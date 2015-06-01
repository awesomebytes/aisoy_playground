#!/usr/bin/env python
# Author: Bence Magyar
from decorator import getfullargspec

import rospy
from sensor_msgs.msg import JointState
from airos4_msgs.srv import GetServoPos, GetServoPosRequest, GetServoPosResponse

if __name__ == "__main__":
    rospy.init_node("joint_state_publisher")

    rospy.wait_for_service('/airos4/servo/get_servo_pos')
    get_servo_state = rospy.ServiceProxy('/airos4/servo/get_servo_pos', GetServoPos)

    pub = rospy.Publisher('/joint_states', JointState, queue_size=100)

    msg = JointState()
    msg.name = ['head_pan_joint', 'head_tilt_joint', 'l_eyebrow_joint', 'r_eyebrow_joint']
    msg.position = [0.0, 0.0, 0.0, 0.0]

    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():

        try:
            msg.position[0] = get_servo_state.call(0).position
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        try:
            msg.position[1] = get_servo_state.call(1).position
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        try:
            msg.position[2] = msg.position[3] = get_servo_state.call(2).position
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        print msg

        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
#        rospy.spinOnce()