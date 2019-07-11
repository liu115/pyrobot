


from config import get_cfg_defaults

_C = get_cfg_defaults()

_C.HAS_BASE = False
_C.HAS_CAMERA = False
_C.HAS_GRIPPER = False

_ARMC = _C.ARM
_ARMC.CLASS = 'TM700Arm'
_ARMC.MOVEGROUP_NAME = 'manipulator'
_ARMC.EE_FRAME = 'tip_link'
_ARMC.ROSTOPIC_SET_JOINT = '/tool_position'

def get_cfg():
    return _C.clone()
