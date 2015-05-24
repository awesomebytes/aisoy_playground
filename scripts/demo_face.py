#!/usr/bin/env python
# Author: Bence Magyar

import rospy
from sensor_msgs.msg import Image
from move_robot import move_servos
import cv2
from cv_bridge import CvBridge

faceCascade = None
bridge = None
state = 0.5
seq = 0
ok_to_move = True

def callback(data):
    global state
    global seq
    global ok_to_move

    image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect faces in the image
    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
        flags = cv2.cv.CV_HAAR_SCALE_IMAGE
    )

    print "Found {0} faces!".format(len(faces))

    print data.header.seq - seq
    seq = data.header.seq

    # Draw a rectangle around the faces
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.circle(image, (x+w/2, y+h/2), 3, (0, 0, 255), 5)
        # a circle only on the Y axis
        cv2.circle(image, (x+w/2, image.shape[0]/2), 3, (0, 0, 255), 2)
        diff = x+w/2 - image.shape[1]/2
        #the ok_to_move filters every second move
        if ok_to_move and len(faces) == 1: #only one face so it wont go crazy with multiple
            if diff > w/2:
                state = state - 0.02
            if diff < -w/2:
                state = state + 0.02
            print "Sending head to " + str(state)
            move_servos(state)
            ok_to_move = False
        else:
            ok_to_move = True

    # don't do this, it spams the service too much
    #if len(faces):
    #    state = 0.5
    #    move_servos(state)

    #a circle in the middle of the image
    cv2.circle(image, (image.shape[1]/2, image.shape[0]/2), 2, (255, 0, 0), 5)

    cv2.imshow("Faces found", image)
    cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("demo_face")

    move_servos(state, None, None)
    print 'Load facestuff'
    faceCascade = cv2.CascadeClassifier('/home/bence/aisoy_ws/src/aisoy_playground/config/haarcascade_frontalface_default.xml')
    bridge = CvBridge()
    sub = rospy.Subscriber("/airos4/camera/image_medium", Image, callback, queue_size=1)
    print 'Subscribed to image'
    rospy.spin()
