#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BoxCentroidLine:
    def __init__(self):
        rospy.init_node('box_centroid_line')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to HSV for color filtering (adjust these ranges as needed)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define color range for detecting the box (example: gray)
        lower_color = np.array([0, 0, 50])
        upper_color = np.array([180, 50, 200])
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Find contours on the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour assuming it's the box
            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) > 1000:
                # Get bounding rect
                x, y, w, h = cv2.boundingRect(largest_contour)

                # Draw bounding box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Calculate centroid of the bounding box
                cx = x + w // 2
                cy = y + h // 2

                # Draw centroid as a small circle
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                # Draw a horizontal line across the bounding box through the centroid
                start_point = (x, cy)
                end_point = (x + w, cy)
                cv2.line(frame, start_point, end_point, (255, 0, 0), 2)

        # Show image
        cv2.imshow("Box with Centroid Line", frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    node = BoxCentroidLine()
    rospy.spin()
