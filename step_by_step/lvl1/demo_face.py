#!/usr/bin/env python
# Author: Bence Magyar

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

# Global variables to store runtime information
faceCascade = None # face detector object
bridge = None # cv bridge object

def callback(data):
    # Use the bridge to convert the ROS Image message to OpenCV message
    image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Pop up a window and visualize the image
    cv2.imshow("Faces found", image)
    cv2.waitKey(1)

    #!!!! Try to visualize the gray image too!

if __name__ == "__main__":
    # Initialize ROS
    rospy.init_node("demo_face")
    # Center the head
    move_head(0.0)
    # Initialize CvBridge which gives us conversion from ROS Image type to OpenCV image type
    bridge = CvBridge()
    # Subscribe to the image, for every new image the function "callback" will be executed once
    sub = rospy.Subscriber("/airos4/camera/image_medium", Image, callback, queue_size=1)
    print 'Subscribed to image'
    rospy.spin() # Go!
