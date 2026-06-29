#!/usr/bin/env python3
"""
Ground Truth TF Publisher
替代 FAST-LIO，直接使用 Gazebo P3D 插件的真值位姿广播 TF (map -> trunk)

工作原理：
  1. 订阅 /ground_truth/odom（Gazebo P3D 插件发布，100% 完美的真值位姿）
  2. 将位姿广播为 TF：map -> trunk
  3. elevation_mapping 和 State_VRL 的 TF 查询无缝对接

使用方法：
  rosrun unitree_guide ground_truth_tf_publisher.py
"""

import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry


class GroundTruthTFPublisher:
    def __init__(self):
        rospy.init_node('ground_truth_tf_publisher', anonymous=False)

        # TF 广播器
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        # 订阅 P3D 插件发布的真值里程计
        # frame_id = "map", child_frame_id = "trunk"
        self._odom_sub = rospy.Subscriber(
            '/ground_truth/odom',
            Odometry,
            self._odom_callback,
            queue_size=10
        )

        self._msg_count = 0
        rospy.loginfo("[GT TF] 真值 TF 发布器已启动，等待 /ground_truth/odom...")

    def _odom_callback(self, msg):
        """将 odometry 消息中的位姿直接广播为 TF"""
        t = geometry_msgs.msg.TransformStamped()

        # 使用里程计消息的时间戳（sim time）
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "map"       # 世界坐标系
        t.child_frame_id = "trunk"     # 机器人躯干坐标系

        # 直接复制真值位姿
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self._tf_broadcaster.sendTransform(t)

        self._msg_count += 1
        if self._msg_count % 200 == 1:  # 约每秒打印一次（200Hz 发布频率）
            rospy.loginfo_throttle(
                1.0,
                "[GT TF] map -> trunk: (%.3f, %.3f, %.3f)",
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z
            )


if __name__ == '__main__':
    try:
        node = GroundTruthTFPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
