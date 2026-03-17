from pathlib import Path
from typing import Final, Tuple
import numpy as np
from ament_index_python.packages import get_package_share_directory
from drc.manipulator.robot_data import RobotData

"""
URDF Joint Information: FR3
Total nq = 7
Total nv = 7

 id | name                 | nq | nv | idx_q | idx_v
----+----------------------+----+----+-------+------
  1 |           fr3_joint1 |  1 |  1 |     0 |    0
  2 |           fr3_joint2 |  1 |  1 |     1 |    1
  3 |           fr3_joint3 |  1 |  1 |     2 |    2
  4 |           fr3_joint4 |  1 |  1 |     3 |    3
  5 |           fr3_joint5 |  1 |  1 |     4 |    4
  6 |           fr3_joint6 |  1 |  1 |     5 |    5
  7 |           fr3_joint7 |  1 |  1 |     6 |    6
"""

TASK_DOF:  Final[int] = 6
JOINT_DOF: Final[int] = 7

class FR3RobotData(RobotData):
    def __init__(self, dt: float) -> None:
        super().__init__(dt            = dt,
                         urdf_path     = str(Path(get_package_share_directory("dyros_robot_menagerie"), "robot", "fr3.urdf")),
                         srdf_path     = str(Path(get_package_share_directory("dyros_robot_menagerie"), "robot", "fr3.srdf")),
                         packages_path = get_package_share_directory("mujoco_ros_sim"),
                        )