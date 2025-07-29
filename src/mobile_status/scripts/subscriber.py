#!/usr/bin/env python3

import rospy
from mobile_status.msg import RobotStatus
from datetime import datetime

def callback(data):
    # Format timestamp
    formatted_time = datetime.fromtimestamp(data.stamp.to_sec()).strftime('%Y-%m-%d %H:%M:%S')
    rospy.loginfo(f"Received: x={data.x}, y={data.y}, dir={data.direction}, time={formatted_time}")

def subscriber():
    rospy.init_node('status_subscriber', anonymous=True)
    rospy.Subscriber('robot_status', RobotStatus, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()

