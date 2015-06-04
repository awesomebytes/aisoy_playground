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
ok_to_move = True # global variable whether it is ok to move the head or not

def callback(data):
    # Magic mantra to access global variables from within function
    global state
    global ok_to_move

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

    # The detected faces are now in 'faces'
    print "Found {0} faces!".format(len(faces))

    # Draw some information related to faces
    # Keep in mind that x,y are the coordinates of the top-left of the rectangle, w,h are width and height respectively
    for (x, y, w, h) in faces:
        # Draw a rectangle around the face
        # http://docs.opencv.org/modules/core/doc/drawing_functions.html?highlight=rectangle#cv2.rectangle
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        # Let's draw a clown-nose for you, circle in the middle of the face
        # http://docs.opencv.org/modules/core/doc/drawing_functions.html?highlight=rectangle#circle
        cv2.circle(image, (x+w/2, y+h/2), 3, (0, 0, 255), 5)
        # Draw circle only on the Y axis, so it is easy to see the difference on the horizontal axis
        # Beware! Image size is different from camera to camera, cannot guarantee the same size
        # Check image.shape to get the center coordinate of the image in the Y axis
        cv2.circle(image, (x+w/2, image.shape[0]/2), 3, (0, 0, 255), 2)


        # Time to turn the head toward the face!

        # We need the distance of the face center from the image center, but let's only use the left-right direction,
        # in other words, the Y axis.

        # The center of the image on the y axis is halfway to the end, use image.shape and
        # keep in mind that you need to use the center of the face, not the corner!
        diff = x+w/2 - image.shape[1]/2

        # Now we can use this difference to move the head left or right.

        # The ok_to_move filters moves, in this case we simply filter every second movement, so that the robot is not
        # overreacting when a face moves.
        # The robot cannot look at multiple faces at the same time, so we only move when there is a single face
        if ok_to_move and len(faces) == 1:
            # If the face is on the right, we move right
            if diff > w/2: # Using w/2 here makes sure that we get the image center into the box of the face
                state = state - 0.02
            # If the face is on the left, we move left
            if diff < -w/2:
                state = state + 0.02
            print "Sending head to " + str(state)
            # Turn the robot head using the move_head function
            move_head(state)
            # Let's not move at all in the next round...
            ok_to_move = False
        else:
            # Ok we didn't move, the next round will be moving
            ok_to_move = True

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
