from config import get_cfg_defaults

_C = get_cfg_defaults()

_C.HAS_BASE = False
_C.HAS_GRIPPER = True

_ARMC = _C.ARM
_ARMC.CLASS = 'TM700Arm'
_ARMC.MOVEGROUP_NAME = 'manipulator'
_ARMC.EE_FRAME = 'tip_link'

_CAMERAC = _C.CAMERA
_CAMERAC.CLASS = 'TM700Camera'
_CAMERAC.ROSTOPIC_CAMERA_INFO_STREAM = '/camera/color/camera_info'
_CAMERAC.ROSTOPIC_CAMERA_RGB_STREAM = '/camera/color/image_raw'
_CAMERAC.ROSTOPIC_CAMERA_DEPTH_STREAM = '/camera/aligned_depth_to_color/image_raw'

_GRIPPERC = _C.GRIPPER
# Gripper class name
_GRIPPERC.CLASS = 'TM700Gripper'
# Topic name to open gripper
_GRIPPERC.ROSTOPIC_GRIPPER_OPEN = '/gripper/open'
# Topic name to close gripper
_GRIPPERC.ROSTOPIC_GRIPPER_CLOSE = '/gripper/close'
# Joint names of the gripper joints
_GRIPPERC.ROSTOPIC_GRIPPER_STATE = '/gripper/state'

def get_cfg():
    return _C.clone()
