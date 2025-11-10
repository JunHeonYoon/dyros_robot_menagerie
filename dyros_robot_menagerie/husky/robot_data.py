from drc import KinematicParam, DriveType
from drc.mobile.robot_data import RobotData

"""
URDF Joint Information: Husky
 name                | value
---------------------+---------------------------
type                 | Differential
wheel_num            | 2
wheel_radius         | 0.1651
base_width           | 1.0702
"""
TASK_DOF  = 3
WHEEL_DOF = 2

class HuskyRobotData(RobotData):
    def __init__(self):
        param = KinematicParam(type         = DriveType.Differential,
                               wheel_radius = 0.1651,
                               base_width   = 0.2854 * 2 * 1.875,
                               max_lin_acc  = 3,
                               max_ang_acc  = 6,
                               )
        super().__init__(param)