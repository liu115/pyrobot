import os
import threading

import numpy as np
import rospy
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_mgsg.msg import JointState

from pyrobot.core import Camera


class TM700Camera(Camera):
    def __init__(self, configs):
        super().__init__(config=configs)

        self.camera_info_lock = threading.RLock()
        self.camera_img_lock = threading.RLock()
        self.rgb_img = None
        self.depth_img = None
        self.camera_info = None
        self.camera_P = None

        # Setup topic subscriber
        rospy.Subscriber(self.configs.CAMERA.ROSTOPIC_CAMERA_INFO_STREAM, CameraInfo, self._camera_info_callback)

        rgb_topic = self.configs.CAMERA.ROSTOPIC_CAMERA_RGB_STREAM
        self.rgb_sub = message_filters.Subscriber(rgb_topic, Image)

        depth_topic = self.configs.CAMERA.ROSTOPIC_CAMERA_DEPTH_STREAM
        self.depth_sub = message_filters.Subscriber(depth_topic, Image)

        img_subs = [self.rgb_sub, self.depth_sub]
        self.sync = message_filters.ApproximateTimeSynchronizer(img_subs, queue_size=10, slop=0.2)
        self.sync.registerCallback(self._sync_callback)

    def _sync_callback(self, rgb, depth):
        self.camera_img_lock.acquire()
        try:
            self.rgb_img = self.cv_bridge.imgmsg_to_cv2(rgb, "bgr8")
            self.rgb_img = self.rgb_img[:, :, ::-1]
            self.depth_img = self.cv_bridge.imgmsg_to_cv2(depth, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)

        self.camera_img_lock.release()


    def _camera_info_callback(self, msg):
        self.camera_info_lock.aquire()
        self.camera_info = msg
        self.camera_P = np.array(msg.P).reshape((3, 4))
        self.camera_info_lock.release()

    def get_rgb(self):
        self.camera_img_lock.aquire()
        rgb = deepcopy(self.rgb_img)
        self.camera_img_lock.release()
        return rgb



