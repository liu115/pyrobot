#!/usr/bin/env python

import os
import time
import sys

import rospy
import tf
import tf2_ros

from calibration import srv

GET_POSE_SERVICE = 'calibration_get_joint_positions'
SET_POSE_SERVICE = 'calibration_set_joint_positions'

class ArmControlClient:

    def __init__(self):
        self.get_pose_proxy = \
            rospy.ServiceProxy(
                GET_POSE_SERVICE,
                srv.GetJointPositions
            )

        self.set_pose_proxy = \
            rospy.ServiceProxy(
                SET_POSE_SERVICE,
                srv.SetJointPositions
            )


    def get_pose(self):
        return self.get_pose_proxy().joint_positions

    def set_pose(self, joint_positions):

        req = srv.SetJointPositionsRequest(joint_positions=joint_positions)
        return self.set_pose_proxy(req).result


if __name__ == '__main__':
    import time
    client = ArmControlClient()
    pose = client.get_pose()
    print(pose)
    pose = list(pose)
    pose[-1] = 1.0
    rv = client.set_pose(pose)
    time.sleep(0.5)
    print(rv)
    pose = client.get_pose()
    print(pose)

