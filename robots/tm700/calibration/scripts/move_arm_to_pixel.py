#!/usr/bin/env python

import io
import os
import threading
import time
import json
from copy import deepcopy

import rospy
import rospkg
from flask import Flask, request
from flask import render_template, send_file, redirect, url_for
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tf
import tf2_ros

from calibration.calibration_arm_control_client import ArmControlClient
print(ArmControlClient)
from easy_handeye.handeye_client import HandeyeClient


app = Flask(__name__)
app.debug = True

# Run ros node on different thread than flask
# threading.Thread(target=lambda: rospy.init_node('test_node', disable_signals=True)).start()
rospy.init_node('test_move_gui_server')

################################
# Setup camera topic
camera_topic = rospy.get_param('camera_topic', '/camera/color/image_raw')
depth_topic = rospy.get_param('depth_topic', '/camera/aligned_depth_to_color/image_raw')

source_frame = None
listener = tf.TransformListener()
target_frame = rospy.get_param('target_frame', 'ar_marker_2')

rgb_img = None
rgb_img_lock = threading.Lock()
depth_img = None
depth_img_lock = threading.Lock()

def rgb_img_callback(data):
    img_data = CvBridge().imgmsg_to_cv2(data, "bgr8")
    rgb_img_lock.acquire()
    global source_frame
    global rgb_img
    source_frame = data.header.frame_id
    rgb_img = deepcopy(img_data)
    rgb_img_lock.release()

def depth_img_callback(data):
    img_data = CvBridge().imgmsg_to_cv2(data, "passthrough")
    depth_img_lock.acquire()
    global depth_img
    depth_img = deepcopy(img_data)
    depth_img_lock.release()

rospy.Subscriber(camera_topic, Image, rgb_img_callback, queue_size=1)
rospy.Subscriber(depth_topic, Image, depth_img_callback, queue_size=1)


def point_msg(p, frame_id):
    ps = PointStamped()
    ps.point.x = p[0]
    ps.point.y = p[1]
    ps.point.z = p[2]
    ps.header = Header()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = frame_id
    return ps

@app.route('/')
def index():
    rv = render_template('move.html')
    a = request.args.get('')
    # ps = point_msg([0, 1, 2], 'base_link')
    # print(ps)
    return rv
    # return 'hello'




intrinsic_mat = [926.739013671875, 0.0, 625.3572387695312, 0.0, 0.0, 925.6869506835938, 350.6984558105469, 0.0, 0.0, 0.0, 1.0, 0.0]
intrinsic_mat = np.array(intrinsic_mat).reshape(3, 4)
intrinsic_mat = intrinsic_mat[:3, :3]
intrinsic_mat_inv = np.linalg.inv(intrinsic_mat)

# Reference:
# https://answers.ros.org/question/146111/tf-python-api-transform-points/
@app.route('/move')
def move():
    q = request.query_string

    try:
        x, y = q.split(',')
        x = int(x)
        y = int(y)
    except Exception as e:
        return str(e)

    with rgb_img_lock:
        with depth_img_lock:
            if rgb_img is None:
                return 'No image'
            if depth_img is None:
                return 'No depth image'

            assert x < rgb_img.shape[1]
            assert y < rgb_img.shape[0]
            xy_one = [x, y, 1]
            z = depth_img[y, x] / 1e3
            xyz = np.dot(intrinsic_mat_inv, xy_one) * z
            msg = point_msg(xyz, source_frame)
            print(msg)

            now = rospy.Time.now()
            listener.waitForTransform('ar_marker_2', source_frame, now, rospy.Duration(10))

            ar_tf = listener.lookupTransform(source_frame, 'ar_marker_2', rospy.Time())
            t = listener.getLatestCommonTime("ar_marker_2", source_frame)
            new_msg = listener.transformPoint('ar_marker_2', msg)

            return str(ar_tf) + '<br />' +  str(xyz) + '<br />' + str([new_msg.point.x, new_msg.point.y, new_msg.point.z])

    return ''

@app.route('/rgb_image.png')
def get_rgb_image():

    # Get image from ros topic
    rgb_img_lock.acquire()
    global rgb_img
    if rgb_img is None:
        return ''
    rv, buffer = cv2.imencode('.png', rgb_img)
    img_file = io.BytesIO(buffer)
    rv = send_file(img_file, attachment_filename='rgb.png', mimetype='image/png')
    rgb_img_lock.release()
    return rv

@app.route('/depth_image.png')
def get_depth_image():

    # Get image from ros topic
    depth_img_lock.acquire()
    global depth_img
    if depth_img is None:
        return ''
    depth_img_out = depth_img.astype(np.float32)
    depth_img_out = np.clip(depth_img_out, 400, 1000) - 400
    depth_img_out = depth_img_out / 600 * 255
    depth_img_out = np.clip(depth_img_out, 0, 255)
    depth_img_out[depth_img_out == 255] = 0
    # depth_img_out /= 10
    depth_img_out = depth_img_out.astype(np.uint8)
    rv, buffer = cv2.imencode('.png', depth_img_out)
    img_file = io.BytesIO(buffer)
    rv = send_file(img_file, attachment_filename='depth.png', mimetype='image/png')
    # cv2.imshow('depth', depth_img)
    # cv2.waitKey(1000)
    depth_img_lock.release()
    return rv



if __name__ == '__main__':
    app.run(port=5000)
