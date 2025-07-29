#!/usr/bin/env python3

import rospy
from mobile_status.msg import RobotStatus
import random
from datetime import datetime

def publisher():
    pub = rospy.Publisher('robot_status', RobotStatus, queue_size=10)
    rospy.init_node('status_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    directions = ['North', 'East', 'South', 'West']

    while not rospy.is_shutdown():
        msg = RobotStatus()
        msg.x = round(random.uniform(0, 100), 2)
        msg.y = round(random.uniform(0, 100), 2)
        msg.direction = random.choice(directions)
        msg.stamp = rospy.Time.now()

        # Format timestamp
        formatted_time = datetime.fromtimestamp(msg.stamp.to_sec()).strftime('%Y-%m-%d %H:%M:%S')
        rospy.loginfo(f"Publishing: x={msg.x}, y={msg.y}, dir={msg.direction}, time={formatted_time}")

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

