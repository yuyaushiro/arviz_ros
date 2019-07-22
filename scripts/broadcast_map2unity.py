#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Quaternion

class BroadcastMap2Unity:
    def __init__(self):
        rospy.init_node('broadcast_map2unity', anonymous=True)
        rospy.Subscriber("unity/robot_pose", PoseStamped,
                         self.robot_pose_callback, queue_size=1)
        self.tf_pub_ = tf.TransformBroadcaster()
        self.tf_sub_ = tf.TransformListener()
        self.mapTfoot = np.identity(4)
        self.unityTfoot = np.identity(4)
        self.mapTunity = np.identity(4)

    def quaternion_to_euler(self, quaternion):
        """Convert Quaternion to Euler Angles

        quarternion: geometry_msgs/Quaternion
        euler: geometry_msgs/Vector3
        """
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y,
                                                      quaternion.z, quaternion.w))
        return e[0], e[1], e[2]
    
    def create_transformation_matrix_yaw(self, x, y, z, yaw):
        return np.array([[np.cos(yaw), -np.sin(yaw), 0, x],
                         [np.sin(yaw),  np.cos(yaw), 0, y],
                         [0          ,  0          , 1, z],
                         [0          ,  0          , 0, 1]])

    def robot_pose_callback(self, data):
        self.tf_sub_.waitForTransform('map', 'base_footprint',
                                      rospy.Time.now(), rospy.Duration(3.0))
        try:
            # map座標系からrobot座標系への同次変換行列
            (trans, rot) = self.tf_sub_.lookupTransform('/map',
                                                        '/base_footprint',
                                                        rospy.Time(0))
            rot_qua = Quaternion()
            rot_qua.x = rot[0]
            rot_qua.y = rot[1]
            rot_qua.z = rot[2]
            rot_qua.w = rot[3]
            roll, pitch, yaw = self.quaternion_to_euler(rot_qua)
            self.mapTfoot = self.create_transformation_matrix_yaw(trans[0],
                                                                trans[1],
                                                                trans[2],
                                                                yaw)

            # unity座標系からrobot座標系への同次変換行列
            d_roll, d_pitch, d_yaw= self.quaternion_to_euler(data.pose.orientation)
            # d_yaw -= np.deg2rad(180)
            self.unityTfoot =\
                self.create_transformation_matrix_yaw(data.pose.position.x,
                                                    data.pose.position.y,
                                                    data.pose.position.z,
                                                    d_yaw)

            # map座標系からunity座標系への同次変換行列
            self.mapTunity = np.matmul(self.mapTfoot, np.linalg.inv(self.unityTfoot))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
                return
        # print "{0: .3f}".format(data.pose.position.x)
        # print "{0: .3f}".format(data.pose.position.y)
        # print "{0: .3f}".format(data.pose.position.z)
        # print '---'
        # print "{0: 3.0f}".format(np.rad2deg(roll))
        # print "{0: 3.0f}".format(np.rad2deg(pitch))
        # print "{0: 3.0f}".format(np.rad2deg(yaw))
        # print '======='
        # print ''
        # print ''
        # print ''

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # map座標系からunity座標系への変換をbroadcaster
            theta = np.arctan2(self.mapTunity[1, 0], self.mapTunity[0, 0])
            self.tf_pub_.sendTransform((self.mapTunity[0, 3], self.mapTunity[1, 3], self.mapTunity[2,3]),
                                        tf.transformations.quaternion_from_euler(0, 0, theta),
                                        rospy.Time.now(),
                                        "unity",
                                        "map")
            rate.sleep()

if __name__ == '__main__':
    broadcast_map2unity = BroadcastMap2Unity()
    broadcast_map2unity.run()
