import numpy as np
import time
import sys
from os.path import expanduser
home = expanduser("~")
sys.path.append(home + '/pyrobot/src')
from pyrobot.core import Robot


def main():
    bot = Robot("tm700",
                use_arm=True,
                use_base=False,
                use_camera=False,
                use_gripper=False)
    bot.arm.go_home()

    group_states = [
            [1.5707, 0,0,0,1.57075,0],
            [1.5707,0,0,-1,5707,1.5707,0],
            [1.5707,0,0,1.5707,-1.5707,0],
            [1.5707,0,0,0,-1.5707,0]
        ]

    for state in group_states:
        bot.arm.set_joint_positions(state)
        time.sleep(1)

    bot.arm.go_home()


if __name__ == "__main__":
    main()
