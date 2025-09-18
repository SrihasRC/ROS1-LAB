#!/usr/bin/env python3
import rospy
from velocity_ramping.msg import Velocity

def main():
    pub = rospy.Publisher('vel_input', Velocity, queue_size=10)
    rospy.init_node('velocity_input_node')

    rate = rospy.Rate(2)  # 2 Hz
    while not rospy.is_shutdown():
        try:
            lin = float(input("Enter linear velocity (x): "))
            ang = float(input("Enter angular velocity (z): "))
            vel = Velocity(linear_x=lin, angular_z=ang)
            pub.publish(vel)
            rospy.loginfo(f"Published input velocity: {vel}")
        except Exception as e:
            rospy.logwarn(f"Invalid input: {e}")
        rate.sleep()

if __name__ == '__main__':
    main()
