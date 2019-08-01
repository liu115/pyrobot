import os
import threading
from copy import deepcopy

import numpy as np
import rospy
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState

from pyrobot.core import Camera


class TM700Camera(Camera):
    def __init__(self, configs):
        super(TM700Camera, self).__init__(configs=configs)
        self.cv_bridge = CvBridge()

        self.camera_info_lock = threading.RLock()
        self.camera_img_lock = threading.RLock()
        self.rgb_img = None
        self.rgb_frame = None
        self.depth_img = None
        self.depth_frame = None
        self.camera_info = None
        self.camera_P = None # projection matrix (combine D and K)
        self.camera_D = None # distortion paramters
        self.camera_K = None # intrinsic camera matrix

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
            self.rgb_frame = rgb.header.frame_id
            self.depth_img = self.cv_bridge.imgmsg_to_cv2(depth, "passthrough")
            self.depth_frame = depth.header.frame_id
        except CvBridgeError as e:
            rospy.logerr(e)

        self.camera_img_lock.release()


    def _camera_info_callback(self, msg):
        self.camera_info_lock.acquire()
        self.camera_info = msg
        self.camera_P = np.array(msg.P).reshape((3, 4))
        self.camera_D = np.array(msg.D)
        self.camera_K = np.array(msg.K).reshape((3, 3))
        self.camera_info_lock.release()

    def get_rgb(self):
        self.camera_img_lock.acquire()
        rgb = deepcopy(self.rgb_img)
        self.camera_img_lock.release()
        return rgb

    def get_rgb_frame(self):
        self.camera_img_lock.acquire()
        frame_id = deepcopy(self.rgb_frame)
        self.camera_img_lock.release()
        return frame_id

    def get_depth(self):
        self.camera_img_lock.acquire()
        depth = deepcopy(self.depth_img)
        self.camera_img_lock.release()
        return depth

    def get_depth_frame(self):
        self.camera_img_lock.acquire()
        frame_id = deepcopy(self.depth_frame)
        self.camera_img_lock.release()
        return frame_id

    def get_rgb_depth(self):
        '''
        This function returns both the RGB and depth
        images perceived by the camera.
        '''
        self.camera_img_lock.acquire()
        rgb = deepcopy(self.rgb_img)
        depth = deepcopy(self.depth_img)
        self.camera_img_lock.release()
        return rgb, depth

    def get_intrinsics(self):
        if self.camera_P is None:
            return self.camera_P
        self.camera_info_lock.acquire()
        P = deepcopy(self.camera_P)
        D = deepcopy(self.camera_D)
        K = deepcopy(self.camera_K)
        self.camera_info_lock.release()
        return P[:3, :3], D, K

    def pix_to_3d(self, x, y):
        '''
        Convert pixel coordinate (x, y) to 3d coordinate based on camera frame
        :return: 3d coordinate numpy array [x, y, z]
        '''
        mat, _, _ = self.get_intrinsics()
        mat_inv = np.linalg.inv(mat)

        depth_img = self.get_depth()
        depth = depth_img[y, x] / 1e3 # convert mm to m

        uv_one = np.array([x, y, 1])
        uv_one_in_cam = np.dot(mat_inv, uv_one)

        xyz = np.multiply(uv_one_in_cam, depth)

        return xyz

