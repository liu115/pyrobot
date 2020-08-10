import numpy as np
import time
import sys
from os.path import expanduser
home = expanduser("~")
sys.path.append(home+ "/pyrobot/src")
from pyrobot.core import Robot



def main():
    bot = Robot('tm700')
    bot.gripper.open()
    time.sleep(3)
    bot.gripper.close()


if __name__ == "__main__":
    main()
