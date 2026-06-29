#!/usr/bin/env python3
"""
FAST-LIO 里程计 → elevation_mapping 位姿话题
将 /Odometry (camera_init 帧) 转为 /base_pose (map 帧)
加上 static TF map->camera_init 的 z=0.405 偏移
"""
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

# map -> camera_init 的静态偏移
Z_OFFSET = 0.405

def odom_callback(msg):
    pose_msg = PoseWithCovarianceStamped()
    # 改 frame_id: camera_init → map
    pose_msg.header = msg.header
    pose_msg.header.frame_id = "map"
    pose_msg.pose = msg.pose
    # 补偿 map -> camera_init 的 z 偏移
    pose_msg.pose.pose.position.z += Z_OFFSET
    pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('odom_to_pose_converter')
    pub = rospy.Publisher('/base_pose', PoseWithCovarianceStamped, queue_size=10)
    rospy.Subscriber('/Odometry', Odometry, odom_callback)
    rospy.spin()
