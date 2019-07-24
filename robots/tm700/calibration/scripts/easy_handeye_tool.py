#!/usr/bin/env python

import io
import os
import threading
import time

import rospy
import rospkg
from flask import Flask, request
from flask import render_template, send_file, redirect, url_for
from sensor_msgs.msg import Image
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
rospy.init_node('calibration_gui_server')

################################
# Setup camera topic
camera_topic = rospy.get_param('camera_topic', '/camera/color/image_raw')
depth_topic = rospy.get_param('depth_topic', '/camera/depth/image_raw')
img = None
img_lock = threading.Lock()
def image_callback(data):
    rgb_img = CvBridge().imgmsg_to_cv2(data, "bgr8")
    img_lock.acquire()
    global img
    rv, buffer = cv2.imencode('.png', rgb_img)
    img = io.BytesIO(buffer)
    img_lock.release()

rospy.Subscriber(camera_topic, Image, image_callback, queue_size=1)
################################
# Setup client

rospy.loginfo("Setting up easy_handeye client, must wait for server services")
handeye_client = HandeyeClient()
handeye_client_lock = threading.Lock()
arm_client = ArmControlClient()
arm_client_lock = threading.Lock()

################################
# Setup tf listener
listener = tf.TransformListener()
robot_base_frame = rospy.get_param('robot_base_frame', '/base_link')
camera_target_frame = rospy.get_param('camera_target_frame', '/ar_marker_2')


def check_marker():
    now = rospy.Time.now()
    listener.waitForTransform(robot_base_frame, camera_target_frame, now, rospy.Duration(0.5))
    tag_tf = listener.lookupTransform(robot_base_frame, camera_target_frame, now)
    return tag_tf

@app.route('/')
def index():
    result = request.args.get('result', None)
    # Get saved samples from Handeye client
    with handeye_client_lock:
        samples = handeye_client.get_sample_list()
        hand_world_samples = samples.hand_world_samples.transforms
        camera_marker_samples = samples.camera_marker_samples.transforms

    with arm_client_lock:
        joints = arm_client.get_pose()

    # Fetch ar_tag frame transform from robot base
    message = None
    tag_tf = None
    tag_state = "Init"
    try:
	tag_tf = check_marker()
        tag_tf = str(tag_tf)
        tag_state = "Found ar_marker"

    except tf.LookupException as e:
        message = str(e)
        tag_state = "No marker detected"
    except tf2_ros.TransformException as e:
        message = str(e)
        tag_state = "No marker now"

    params = {
        'tag_tf': tag_tf,
        'tag_state': tag_state,
        'message': message,
        'result': result,
        'hand_world_samples': hand_world_samples,
        'camera_marker_samples': camera_marker_samples,
        'joints': joints
    }
    rv = render_template('index.html', **params)
    # rv = render_template('index.html', tag_tf=tag_tf, tag_state=tag_state, message=message)

    return rv

@app.route('/image.png')
def get_image():

    # Get image from ros topic
    img_lock.acquire()
    rv = send_file(img, attachment_filename='image.png', mimetype='image/png')
    img_lock.release()
    return rv

@app.route('/tf/tag')
def get_tf_tag():
    # Get tf frame tag from tracker (alvar)
    return ''

@app.route('/take_sample')
def take_sample():
    ANGLE_DIFF = np.radians(15)
    # 1. set up lock
    # 2. handeye_client.get_sample()
    # 3. redirect to index
    with arm_client_lock:
        joints = arm_client.get_pose()
        last_joint = joints[-1]

    angles = np.linspace(-ANGLE_DIFF, ANGLE_DIFF, 10)
    angles = last_joint + angles

    with handeye_client_lock:
        for angle in angles:

            with arm_client_lock:
                joints = arm_client.get_pose()
                joints = list(joints)
                joints[-1] = angle

                arm_client.set_pose(joints)
                time.sleep(0.2)

            try:
		tag_tf = check_marker()
		samples = handeye_client.take_sample()
	    except tf2_ros.TransformException as e:
		print(e)

        with arm_client_lock:
            joints = arm_client.get_pose()
            joints = list(joints)
            joints[-1] = last_joint
            arm_client.set_pose(joints)

    return redirect('/')

@app.route('/reset')
def reset():
    with handeye_client_lock:
        samples = handeye_client.get_sample_list()
        for i in range(len(samples.hand_world_samples.transforms)):
            handeye_client.remove_sample(0)

    return redirect('/')

@app.route('/compute')
def compute_calibration():
    with handeye_client_lock:
        result = handeye_client.compute_calibration()
        result = result.calibration.transform.transform
    return redirect(url_for('index', result=result))

@app.route('/save')
def save_result():
    result = request.args.get('result', None)
    with handeye_client_lock:
        handeye_client.save()

    return redirect(url_for('index', result=result))
if __name__ == '__main__':
    app.run(port=5000)
