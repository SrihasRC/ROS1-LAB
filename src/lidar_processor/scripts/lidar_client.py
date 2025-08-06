#!/usr/bin/env python3
import rospy
import random
from lidar_service.srv import LidarProcess

def generate_lidar_data():
    return [random.randint(1, 10) for _ in range(20)]

def call_server(robot_id):
    rospy.wait_for_service('process_lidar')
    try:
        process = rospy.ServiceProxy('process_lidar', LidarProcess)
        data = generate_lidar_data()
        response = process(data, robot_id)
        rospy.loginfo(f"[Client {robot_id}] Sent data: {data}")
        rospy.loginfo(f"[Client {robot_id}] Obstacles detected: {response.obstacle_count}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('lidar_client')
    robot_id = rospy.get_param("~robot_id", random.randint(1, 100))
    call_server(robot_id)
