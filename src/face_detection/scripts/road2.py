#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class RotatedBoxCentroidLine:
    def __init__(self):
        rospy.init_node('rotated_box_centroid_line')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Adjust these to your object color range
        lower_color = np.array([0, 0, 50])
        upper_color = np.array([180, 50, 200])
        mask = cv2.inRange(hsv, lower_color, upper_color)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) > 1000:
                # Get rotated rectangle from contour
                rect = cv2.minAreaRect(largest_contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                # Draw rotated rectangle
                cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

                # Extract centroid
                cx, cy = map(int, rect[0])

                # Draw centroid
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                # Get box width and height
                width, height = rect[1]

                # Compute the vector along the width to draw horizontal line inside the rotated box
                # Get unit vector along width
                angle = rect[2]
                if width < height:
                    angle = angle + 90  # Adjust angle if needed

                rad = np.deg2rad(angle)
                dx = int(np.cos(rad) * width / 2)
                dy = int(np.sin(rad) * width / 2)

                # Calculate start and end points of the horizontal line through centroid
                start_point = (cx - dx, cy - dy)
                end_point = (cx + dx, cy + dy)

                cv2.line(frame, start_point, end_point, (255, 0, 0), 2)

        cv2.imshow("Rotated Box with Centroid Line", frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    node = RotatedBoxCentroidLine()
    rospy.spin()
