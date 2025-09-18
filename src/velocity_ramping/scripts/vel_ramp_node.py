#!/usr/bin/env python3
import rospy
from velocity_ramping.msg import Velocity
from geometry_msgs.msg import Twist

STEP = 5.0
current_lin = 0.0
current_ang = 0.0

pub_cmd = None
pub_updated = None

def ramp(old, new, step):
    """Bounded step ramp function."""
    diff = new - old
    if abs(diff) <= step:
        return new
    return old + step if diff > 0 else old - step

def ramp_callback(msg):
    global current_lin, current_ang
    old_lin, old_ang = current_lin, current_ang

    # apply bounded step ramping
    current_lin = ramp(old_lin, msg.linear_x, STEP)
    current_ang = ramp(old_ang, msg.angular_z, STEP)

    # log
    rospy.loginfo(f"Old vel=({old_lin}, {old_ang}) -> New vel=({current_lin}, {current_ang})")

    # publish Twist for turtlesim
    twist = Twist()
    twist.linear.x = current_lin
    twist.angular.z = current_ang
    pub_cmd.publish(twist)

    # publish updated Velocity msg
    updated = Velocity(linear_x=current_lin, angular_z=current_ang)
    pub_updated.publish(updated)

def main():
    global pub_cmd, pub_updated
    rospy.init_node('velocity_ramp_node')

    rospy.Subscriber('vel_input', Velocity, ramp_callback)
    pub_cmd = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub_updated = rospy.Publisher('vel_updated', Velocity, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()
