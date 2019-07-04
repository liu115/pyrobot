


import numpy as np
import rospy
from collections import namedtuple
from pyrobot.core import Arm

class TM700Arm(Arm):
    """
    This class has the functionality to control a tm700 manipulator
    """

    def __init__( self,
                  configs,
                  moveit_planner='ESTkConfigDefault'):

        super(TM700Arm, self).__init__( configs = configs,
                                        moveit_planner = moveit_planner,
                                        use_moveit() = True)

    def moveto_group_state(self, state: str):
        """
        Command robot to group state with default at home
        """
        tm700_joints = np.zeros(self.arm_dof)
        e_j1, s_j1, s_j2, w_j1, w_j2, w_j3 = tm700_joints

        if str != "home":
            state_num = int(state[5:])

            if state_num == 1:
                e_j1, w_j2 = 1.5707
                if state_num == 2:
                    w_j1 = -1.5707
                    if state_num == 3:
                    w_j1, w_j2 = w_j2, w_j1
                if state_num == 4:
                    w_j2 = -1.5707

        joint_pos = [e_j1, s_j1, s_j2, w_j1, w_j2, w_j3]
        self.set_joint_positions(joint_pos, plan=True)






