import numpy as np
import time
import sys
from os.path import expanduser
home = expanduser("~")
sys.path.append(home + '/pyrobot/src')
from pyrobot.core import Robot


def main():
    target_poses = [{'position': np.array([0.8219, 0.0239, 0.0996]),
                    'orientation': np.array([[-0.3656171, 0.6683861, 0.6477531],
                                            [0.9298826, 0.2319989, 0.2854731],
                                            [0.0405283, 0.7067082, -0.7063434]]
                                            )},
                    {'position': np.array([0.7320, 0.1548, 0.0768]),
                    'orientation': np.array([0.1817, 0.9046, -0.1997, 0.3298])},]

    bot = Robot("tm700",
                use_arm=True,
                use_base=False,
                use_camera=False,
                use_gripper=False)
    bot.arm.go_home()

    for pose in target_poses:
        bot.arm.set_ee_pose(plan=True, **pose)
        time.sleep(1)


if __name__ == "__main__":
    main()
