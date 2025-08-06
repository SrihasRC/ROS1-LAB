#!/usr/bin/env python3
from lidar_service.srv import LidarProcess, LidarProcessResponse
import rospy

# Simulate obstacle detection within 2 to 8 meters
MIN_DIST = 2
MAX_DIST = 8

def handle_lidar(req):
    count = sum(MIN_DIST <= dist <= MAX_DIST for dist in req.lidar_readings)
    rospy.loginfo(f"[Server] Robot {req.robot_id} sent {len(req.lidar_readings)} readings. Obstacles in range: {count}")
    return LidarProcessResponse(obstacle_count=count, robot_id=req.robot_id)

def lidar_server():
    rospy.init_node('lidar_server')
    service = rospy.Service('process_lidar', LidarProcess, handle_lidar)
    rospy.loginfo("Lidar Server ready...")
    rospy.spin()

if __name__ == "__main__":
    lidar_server()
