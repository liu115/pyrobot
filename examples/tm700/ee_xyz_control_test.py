from pyrobot.core import Robot
import numpy as np
import time
import sys
sys.path.append('/home/bfy2101/pyrobot/src')


def main():
    np.set_printoptions(precision=4, suppress=True)

    bot = Robot("tm700",
                use_arm=True,
                use_base=False,
                use_camera=False,
                use_gripper=False)
    bot.arm.go_home()

    displacement = np.array([0.15, 0, 0],
                            [0., 0.15, 0]
                            [0., 0., 0.15])
    for delta in displacement:
        bot.arm.move_ee_xyz(delta, plan=True)
        time.sleep(1)


if __name__ == "__main__":
    main()
