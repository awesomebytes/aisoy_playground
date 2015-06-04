#!/usr/bin/env python
# Author: Bence Magyar

import rospy
from sensor_msgs.msg import Image
from move_robot_nao import move_head
import cv2
from cv_bridge import CvBridge
from rospkg import RosPack

# Global variables to store runtime information
bridge = None # cv bridge object

def callback(data):
    # Use the bridge to convert the ROS Image message to OpenCV message
    image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Let's mark the middle of the image with a blue circle
    # http://docs.opencv.org/modules/core/doc/drawing_functions.html?highlight=rectangle#circle
    # PUT YOUR CODE HERE

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
    sub = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, callback, queue_size=1)
    print 'Subscribed to image'
    rospy.spin() # Go!
