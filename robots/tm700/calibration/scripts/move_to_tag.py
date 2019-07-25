#!/usr/bin/env python
import time
import sys
import os

import rospy
import tf
import tf2_ros
import numpy as np
import geometry_msgs.msg

rospy.init_node('test_move')

listener = tf.TransformListener()
time.sleep(1)

camera_frame = 'camera_link'
marker_frame = 'ar_marker_1'
end_frame = 'tip_link'
base_frame = 'base_link'


def get_tf(src_frame, dst_frame):
    now = rospy.Time.now()
    listener.waitForTransform(src_frame, dst_frame, now, rospy.Duration(1))
    rv = listener.lookupTransform(src_frame, dst_frame, now)

    print(rv)
    return rv

mark_pose = get_tf(base_frame, marker_frame)
# get_tf(base_frame, end_frame)

# rot = np.eye(4)
# rot[2, 2] = np.pi

def invert_pose(quat):
    quat = np.array(quat)
    rot = tf.transformations.euler_matrix(0, np.pi, 0)
    rot = tf.transformations.quaternion_from_matrix(rot)
    quat = tf.transformations.quaternion_multiply(rot, quat)
    return quat


quat = invert_pose(mark_pose[1])

def send_static_tf(trans, orient):
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    static_tf = geometry_msgs.msg.TransformStamped()
    static_tf.header.stamp = rospy.Time.now()
    static_tf.header.frame_id = base_frame # src frame (base_link)
    static_tf.child_frame_id = 'new_marker' # dst frame

    static_tf.transform.translation.x = trans[0]
    static_tf.transform.translation.y = trans[1]
    static_tf.transform.translation.z = trans[2]

    static_tf.transform.rotation.x = orient[0]
    static_tf.transform.rotation.y = orient[1]
    static_tf.transform.rotation.z = orient[2]
    static_tf.transform.rotation.w = quat[3]
    broadcaster.sendTransform(static_tf)
    rospy.spin()

path = os.path.expanduser('~/pyrobot/src')
sys.path.append(path)
from pyrobot.core import Robot

bot = Robot('tm700')
pose = bot.arm.get_ee_pose(base_frame)
origin_pose = [pose[0], pose[2]]
print('origin pose:', origin_pose)

target_pose = [np.array(mark_pose[0]), quat]
target_pose[0][2] = 0.15
print('target pose:', target_pose)

bot.arm.set_ee_pose(target_pose[0], target_pose[1])

# Set same orientation
# bot.arm.set_ee_pose(pose[0], np.array(mark_pose[1]))

# Set x,y translation
# target = np.array([mark_pose[0][0], mark_pose[0][1], 0.25])
# bot.arm.set_ee_pose(target, np.array(mark_pose[1]))

