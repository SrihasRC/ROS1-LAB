#!/usr/bin/env python3
import rospy
from custom_msgs.msg import Person

def callback(msg):
    rospy.loginfo("Received -> Name: %s | Age: %d | Height: %.2f | Weight: %.2f | Present: %s",
                  msg.name, msg.age, msg.height, msg.weight, msg.isPresent)


def listener():
    rospy.init_node('chat_listener', anonymous=True)
    rospy.Subscriber('chat_topic', Person, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
