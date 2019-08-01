import io
import os
import time
import sys

import rospy
import cv2
import numpy as np
import tf
import tf2_ros
from flask import Flask, request
from flask import render_template, send_file, redirect, url_for

rospy.init_node('test_move_gui_server')

pyrobot_path = os.path.expanduser("~/pyrobot/src")
sys.path.append(pyrobot_path)
from pyrobot.core import Robot
from pyrobot import util
bot = Robot("tm700")

app = Flask(__name__)
app.debug = True

listener = tf.TransformListener()
target_frame = rospy.get_param('target_frame', 'base_link')


@app.route('/')
def index():
    rv = render_template('move.html')
    return rv


@app.route('/move')
def move():
    # Get x, y from request
    q = request.query_string
    try:
        x, y = q.split(',')
        x = int(x)
        y = int(y)
    except Exception as e:
        return str(e)


    # Get images
    rgb_img, depth_img = bot.camera.get_rgb_depth()

    assert x < rgb_img.shape[1]
    assert y < rgb_img.shape[0]
    source_frame = bot.camera.get_rgb_frame()
    xyz = bot.camera.pix_to_3d(x, y)
    new_xyz = util.transform_point_between_frames(listener, xyz, source_frame, target_frame)

    return str(xyz) + '<br/>' + str(new_xyz)

@app.route('/rgb_image.png')
def get_rgb_image():
    rgb_img = bot.camera.get_rgb()[:, :, ::-1]
    print(rgb_img)
    rv, buffer = cv2.imencode('.png', rgb_img)
    img_file = io.BytesIO(buffer)
    rv = send_file(img_file, attachment_filename='rgb.png', mimetype='image/png')
    return rv

@app.route('/depth_image.png')
def get_depth_image():
    depth_img = bot.camera.get_depth()

    depth_img_out = depth_img.astype(np.float32)
    depth_img_out = np.clip(depth_img_out, 400, 1000) - 400
    depth_img_out = depth_img_out / 600 * 255
    depth_img_out = np.clip(depth_img_out, 0, 255)
    depth_img_out[depth_img_out == 255] = 0
    depth_img_out = depth_img_out.astype(np.uint8)

    rv, buffer = cv2.imencode('.png', depth_img_out)
    img_file = io.BytesIO(buffer)
    rv = send_file(img_file, attachment_filename='depth.png', mimetype='image/png')
    return rv


if __name__ == '__main__':
    app.run(port=5000)
