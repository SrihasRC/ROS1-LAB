#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FaceDetector:
    def __init__(self):
        rospy.init_node('face_detector')
        self.bridge = CvBridge()
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
    
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale for face detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect faces
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30,30))

        # Draw bounding boxes
        for (x, y, w, h) in faces:
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Show the result
        cv2.imshow('Face Detection', cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    fd = FaceDetector()
    rospy.spin()
