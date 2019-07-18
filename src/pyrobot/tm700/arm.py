


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
                                        use_moveit=True )

    def go_home(self):
        self.set_joint_positions(np.zeros(self.arm_dof), plan= True)

    






