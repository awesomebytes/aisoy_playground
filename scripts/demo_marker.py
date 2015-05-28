#!/usr/bin/env python
# Author: Bence Magyar

import rospy
from std_msgs.msg import UInt32MultiArray
from move_robot import move_head
from led_robot import set_color
from airos4_msgs.msg import Touch

def vision_callback(markers):
#    print markers.data

    if 26 in markers.data:
        print "Robot is now afraid"
        move_head(eyelid=1.0, eyebrow=1.0)
        set_color(20, 0, 5)
    if 582 in markers.data:
        print "Robot is now sleepy"
        move_head(eyelid=0.5, eyebrow=0.5)
        set_color(0, 0, 25)


#rosservice call /airos4/performance/set_performance "name: 'angry'
#perform: true"
# worked so far: happy, angry, scared

def touch_callback(data):
    if data.head or data.left or data.right:
        print "Robot is now happy"
        move_head(eyelid=1.0, eyebrow=0.5)
        set_color(0, 50, 0)

if __name__ == "__main__":
    rospy.init_node("demo_face")

    move_head(0.5, 1.0, 0.5)
    set_color(10, 10, 10)
    vision_sub = rospy.Subscriber("/aruco/markers_list", UInt32MultiArray, vision_callback, queue_size=1)
    touch_sub = rospy.Subscriber("/airos4/touch/touch", Touch, touch_callback, queue_size=1)
    rospy.spin()
