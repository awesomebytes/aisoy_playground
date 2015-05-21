#!/usr/bin/env python
# Author: Sammy Pfeiffer

import rospy
from dynamic_reconfigure.server import Server
from aisoy_playground.cfg import ServosConfig
from move_robot import move_servos

def callback(config, level):
    """
    :param config:
    :param level:
    :return:
    """
    #rospy.loginfo("Received reconf call: " + str(config))
    head_pan = config['head_pan']
    eyelid = config['eyelid']
    eyebrow = config['eyebrow']
    rospy.loginfo("Sending servos (head_pan, eyelid, eyebrow) to " + str((head_pan, eyelid, eyebrow)))
    move_servos(head_pan, eyelid, eyebrow)

    return config

if __name__ == "__main__":
    rospy.init_node("servos_dyn_reconf")

    srv = Server(ServosConfig, callback)
    rospy.spin()