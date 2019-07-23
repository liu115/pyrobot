#!/usr/bin/env python

import io
import os
import threading
import time

import rospy
import rospkg
from flask import Flask, render_template, send_file, redirect, url_for, request
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tf
import tf2_ros
import sys
from os.path import expanduser
home = expanduser("~")
sys.path.append(home + '/pyrobot/src')
from pyrobot.core import Robot
from easy_handeye.handeye_client import HandeyeClient



app = Flask(__name__)
app.debug = True

# Run ros node on different thread than flask
rospy.init_node('test_node')
# threading.Thread(target=lambda: rospy.init_node('test_node', disable_signals=True)).start()

# publisher = rospy.Publisher('test_pub', String, queue_size=1)

bot = Robot("tm700",
                use_arm=True,
                use_base=False,
                use_camera=False,
                use_gripper=False)
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
client = HandeyeClient()
client_lock = threading.Lock()

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
    with client_lock:
        samples = client.get_sample_list()
        hand_world_samples = samples.hand_world_samples.transforms
        camera_marker_samples = samples.camera_marker_samples.transforms

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
        'camera_marker_samples': camera_marker_samples
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
    # 1. set up lock
    # 2. client.get_sample()
    # 3. redirect to index
    last_joint = bot.arm.get_joint_angle('wrist_3_joint')
    ANGLE_DIFF = np.radians(15)
    angles = np.linspace(-ANGLE_DIFF, ANGLE_DIFF, 10)
    angles = last_joint + angles

    with client_lock:
        for angle in angles:
            joint_pos = bot.arm.get_joint_angles()
            joint_pos[-1] = angle
            bot.arm.set_joint_positions(joint_pos)
            time.sleep(0.2)
	
	    try:
		tag_tf = check_marker()
		samples = client.take_sample()
	    except tf2_ros.TransformException as e:
		print(e)
        joint_pos = bot.arm.get_joint_angles()
        joint_pos[-1] = last_joint
        bot.arm.set_joint_positions(joint_pos)
    return redirect('/')

@app.route('/reset')
def reset():
    with client_lock:
        samples = client.get_sample_list()
        for i in range(len(samples.hand_world_samples.transforms)):
            client.remove_sample(0)

    return redirect('/')

@app.route('/compute')
def compute_calibration():
    with client_lock:
        result = client.compute_calibration()
        result = result.calibration.transform.transform
    return redirect(url_for('index', result=result))

@app.route('/save')
def save_result():
    result = request.args.get('result', None)
    with client_lock:
        client.save()

    return redirect(url_for('index', result=result))
if __name__ == '__main__':
    app.run(port=5000)
