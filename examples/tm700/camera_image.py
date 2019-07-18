import sys
from os.path import expanduser
home = expanduser("~")
sys.path.append(home + '/pyrobot/src')

import cv2
import numpy as np
from pyrobot.core import Robot

if __name__ == '__main__':

    bot = Robot('tm700')
    rgb, depth = bot.camera.get_rgb_depth()
    depth = depth.astype(np.float64) / 1e3

    cv2.imshow('Color', rgb[:, :, ::-1])
    cv2.imshow('Depth', depth)

    cv2.waitKey(5000)
