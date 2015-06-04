#!/usr/bin/env python
# Author: Bence Magyar

import rospy
from std_msgs.msg import UInt32MultiArray
from move_robot import move_head
from led_robot import set_color # Do robots dream of many leds?

def vision_callback(markers):
    # When the robot sees different markers, it changes the behaviour.
    # Inspect rostopic echo /aruco/markers_list
    # print markers.data
    pass

if __name__ == "__main__":
    # Initialize ROS
    rospy.init_node("demo_face")
    # Move head to center and open eyes
    move_head(0.5, 1.0, 0.5)
    # Set some led colors
    set_color(10, 10, 10)
    # Subscribe to aruco markers
    vision_sub = rospy.Subscriber("/aruco/markers_list", UInt32MultiArray, vision_callback, queue_size=1)
    rospy.spin() # Go!
