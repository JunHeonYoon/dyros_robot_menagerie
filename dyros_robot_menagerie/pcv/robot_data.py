from drc import KinematicParam, DriveType
from drc.mobile.robot_data import RobotData
import numpy as np

"""
URDF Joint Information: PCV
 name                | value
---------------------+---------------------------
type                 | Caster
wheel_num            | 8
wheel_radius         | 0.0550
offset               | 0.0200

base2wheel_positions
 idx | position
-----+-------------------------
   0 | [0.2150  0.1250]
   1 | [ 0.2150  -0.1250]
   2 | [-0.2150   0.1250]
   3 | [-0.2150  -0.1250]
"""
TASK_DOF  = 3
WHEEL_DOF = 8

class PCVRobotData(RobotData):
    def __init__(self):
        param = KinematicParam(type                 = DriveType.Caster,
                               wheel_radius         = 0.055,
                               wheel_offset         = 0.020,
                               base2wheel_positions = [np.array([0.215, 0.125]),
                                                       np.array([0.215, -0.125]),
                                                       np.array([-0.215, 0.125]),
                                                       np.array([-0.215, -0.125])],
                               max_lin_acc          = 3,
                               max_ang_acc          = 6,
                               )
        super().__init__(param)