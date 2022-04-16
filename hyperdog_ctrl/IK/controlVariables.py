

from matplotlib.cbook import file_requires_unicode
import numpy as np




MIN_LEG_HEIGHT = 80
MAX_LEG_HEIGHT = 240
MAX_ROLL       = 45
MAX_PITCH      = 45
MAX_YAW        = 50 



class RobotState():
    start = False
    walk  = False
    side_move_mode = 0
    height = MIN_LEG_HEIGHT
    eular_ang = [0, 0, 0]
    zeroPnts = [[0,0], [0,0], [0,0], [0,0]]
    speed = 0

class GaitParameters():
    step_length = [0, 0]
    step_height = 0
    step_depth = 0
    freq       = 0
    swing_time = 0
    start_pnt  = [0, 0]
    end_pnt    = [0, 0]

class LegParameters():
    zeroPnt = [0,0]
    cur_pose = [0, 0, 0]
    target_pose = [0,0, 0]
    z_err    = 0

class LegsParameters():
    FR = LegParameters()
    FL = LegParameters()
    BR = LegParameters()
    BL = LegParameters()

class JointAngles():
    cur_angle    = [0, 0, 0]
    target_angle = [0, 0, 0]




        
        



