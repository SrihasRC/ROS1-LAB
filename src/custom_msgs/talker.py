#!/usr/bin/env python3
import rospy
from custom_msgs.msg import Person

def talker():
    pub = rospy.Publisher('chat_topic', Person, queue_size=10)
    rospy.init_node('chat_talker', anonymous=True)

    while not rospy.is_shutdown():
        name = input("Enter your name: ")
        age = int(input("Enter your age: "))
        height = float(input("Enter your height: "))
        weight = float(input("Enter your weight: "))
        isPresent = input("Are you present? (yes/no): ").strip().lower() in ['yes', 'y', 'true', '1']

        msg = Person()
        msg.name = name
        msg.age = age
        msg.height = height
        msg.weight = weight
        msg.isPresent = isPresent

        pub.publish(msg)
        rospy.loginfo("Sent -> Name: %s | Age: %d | Height: %.2f | Weight: %.2f | Present: %s",
              name, age, height, weight, isPresent)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
