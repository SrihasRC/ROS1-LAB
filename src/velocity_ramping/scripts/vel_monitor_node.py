#!/usr/bin/env python3
import rospy
from velocity_ramping.msg import Velocity

def callback(msg):
    rospy.loginfo(f"Monitored Updated Velocity: linear={msg.linear_x}, angular={msg.angular_z}")

def main():
    rospy.init_node('velocity_monitor_node')
    rospy.Subscriber('vel_updated', Velocity, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
