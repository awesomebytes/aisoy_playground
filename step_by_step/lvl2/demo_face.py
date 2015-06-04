#!/usr/bin/env python
# Author: Bence Magyar

import rospy
from sensor_msgs.msg import Image
from move_robot import move_head
import cv2
from cv_bridge import CvBridge
from rospkg import RosPack

# Global variables to store runtime information
faceCascade = None # face detector object
bridge = None # cv bridge object
state = 0.5 # neck motor state

def callback(data):
    # Use the bridge to convert the ROS Image message to OpenCV message
    image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    # Convert the image to grayscale, the face detector doesn't support colors
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect faces in the image
    # Full description:
    # http://docs.opencv.org/modules/objdetect/doc/cascade_classification.html?highlight=detectmultiscale#cascadeclassifier-detectmultiscale
    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
        flags = cv2.cv.CV_HAAR_SCALE_IMAGE
    )

    # The detected faces are now in 'faces', print how many are there!
    # YOUR CODE HERE
    pass

    # Draw some information related to faces
    # Keep in mind that x,y are the coordinates of the top-left of the rectangle, w,h are width and height respectively
    for (x, y, w, h) in faces:
        # Draw a rectangle around the face
        # http://docs.opencv.org/modules/core/doc/drawing_functions.html?highlight=rectangle#cv2.rectangle
        # YOUR CODE HERE
        pass

        # Let's draw a clown-nose for you, circle in the middle of the face
        # http://docs.opencv.org/modules/core/doc/drawing_functions.html?highlight=rectangle#circle
        # YOUR CODE HERE
        pass

    # Let's mark the middle of the image with a blue circle
    cv2.circle(image, (image.shape[1]/2, image.shape[0]/2), 2, (255, 0, 0), 5)

    # Pop up a window and visualize the image
    cv2.imshow("Faces found", image)
    cv2.waitKey(1)

if __name__ == "__main__":
    # Initialize ROS
    rospy.init_node("demo_face")
    # Center the head
    move_head(state)
    print 'Load face detector'
    rp = RosPack() # utility to find path of ROS packages from code
    # Initialize OpenCV face detector with the offline trained dataset
    faceCascade = cv2.CascadeClassifier(rp.get_path('aisoy_playground')+'/config/haarcascade_frontalface_default.xml')
    # Initialize CvBridge which gives us conversion from ROS Image type to OpenCV image type
    bridge = CvBridge()
    # Subscribe to the image, for every new image the function "callback" will be executed once
    sub = rospy.Subscriber("/airos4/camera/image_medium", Image, callback, queue_size=1)
    print 'Subscribed to image'
    rospy.spin() # Go!
