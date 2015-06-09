#!/usr/bin/env python
# Author: Bence Magyar

import rospy
from std_msgs.msg import UInt32MultiArray
from aisoy_playground.move_robot import move_head
from aisoy_playground.led_robot import set_color # Do robots dream of many leds?
from airos4_msgs.msg import Touch

def vision_callback(markers):
    # When the robot sees different markers, it changes the behaviour.
    # Inspect rostopic echo /aruco/markers_list
    # print markers.data
    if 26 in markers.data:
        # To make an afraid face command the eyelids and eyebrows and set the color to something strong
        print "Robot is now afraid"
        move_head(eyelid=1.0, eyebrow=1.0)
        set_color(20, 0, 5)
    if 582 in markers.data:
        # To make a sleepy face command the eyelids and eyebrows and set the color to something peaceful
        print "Robot is now sleepy"
        move_head(eyelid=0.5, eyebrow=0.5)
        set_color(0, 0, 25)

# Aisoy also has "performances", try using them:
#rosservice call /airos4/performance/set_performance "name: 'angry'
#perform: true"
# Worked so far: happy, angry, scared

def touch_callback(data):
    # When the robot is touched on the head, on the left or right,
    # it makes a happy face and led turns green.
    # Check rostopic echo /airos4/touch/touch
    if data.head or data.left or data.right:
        print "Robot is now happy"
        move_head(eyelid=1.0, eyebrow=0.5)
        set_color(0, 50, 0)

if __name__ == "__main__":
    # Initialize ROS
    rospy.init_node("demo_face")
    # Move head to center and open eyes
    move_head(0.5, 1.0, 0.5)
    # Set some led colors
    set_color(10, 10, 10)
    # Subscribe to aruco markers
    vision_sub = rospy.Subscriber("/aruco/markers_list", UInt32MultiArray, vision_callback, queue_size=1)
    # Subscribe to touch feedback
    touch_sub = rospy.Subscriber("/airos4/touch/touch", Touch, touch_callback, queue_size=1)
    rospy.spin() # Go!
