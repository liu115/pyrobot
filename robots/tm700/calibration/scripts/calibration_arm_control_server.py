#!/usr/bin/env python

import os
import threading
import time
import sys

import rospy
import tf
import tf2_ros

from calibration import srv

rospy.init_node('calibration_arm_control_server')

# Use pyrobot as control
from os.path import expanduser
home = expanduser("~")
sys.path.append(home + '/pyrobot/src')
from pyrobot.core import Robot

GET_POSE_SERVICE = 'calibration_get_joint_positions'
SET_POSE_SERVICE = 'calibration_set_joint_positions'

class ArmControlServer:

    def __init__(self):
        self.bot = Robot("tm700")
        self.get_pose_service = \
            rospy.Service(
                GET_POSE_SERVICE,
                srv.GetJointPositions,
                self.get_pose
            )

        self.set_pose_service = \
            rospy.Service(
                SET_POSE_SERVICE,
                srv.SetJointPositions,
                self.set_pose
            )


    def get_pose(self, _):
        rospy.loginfo("Recieve get pose request")
        joints = self.bot.arm.get_joint_angles()
        res = srv.GetJointPositionsResponse()
        res.joint_positions = tuple(joints)
        return res


    def record_pose(self):
        pass

    def set_pose(self, req):
        rospy.loginfo("Recieve set pose request")
        # TODO: check legal positions (prevent hit table)

        joints = req.joint_positions
        try:
            self.bot.arm.set_joint_positions(joints)
        except Exception as e:
            print(e)
            return srv.SetJointPositionsResponse(False)
        return srv.SetJointPositionsResponse(True)

if __name__ == '__main__':
    server = ArmControlServer()
    rospy.spin()
