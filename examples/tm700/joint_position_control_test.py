import time

from pyrobot import Robot


def main():
    bot = Robot( "tm700",
                  use_arm = False,
                  use_camera = False,
                  use_gripper = False )

    group_states = ["home", "ready1", "ready2", "ready3", "ready4"]

    for state in group_states:
        bot.moveto_group_state(state)
        time.sleep(1)

if __name__ == "__main__":
    main()
