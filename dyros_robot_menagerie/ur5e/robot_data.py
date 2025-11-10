from pathlib import Path
from typing import Final, Tuple
import numpy as np
from ament_index_python.packages import get_package_share_directory
from drc.manipulator.robot_data import RobotData

"""
URDF Joint Information: UR5e
Total nq = 6
Total nv = 6

 id | name                 | nq | nv | idx_q | idx_v
----+----------------------+----+----+-------+------
  1 |   shoulder_pan_joint |  1 |  1 |     0 |    0
  2 |  shoulder_lift_joint |  1 |  1 |     1 |    1
  3 |          elbow_joint |  1 |  1 |     2 |    2
  4 |        wrist_1_joint |  1 |  1 |     3 |    3
  5 |        wrist_2_joint |  1 |  1 |     4 |    4
  6 |        wrist_3_joint |  1 |  1 |     5 |    5
"""

TASK_DOF:  Final[int] = 6
JOINT_DOF: Final[int] = 6

class UR5eRobotData(RobotData):
    def __init__(self) -> None:
        super().__init__(urdf_path     = str(Path(get_package_share_directory("dyros_robot_menagerie"), "robot", "ur5e.urdf")),
                         srdf_path     = str(Path(get_package_share_directory("dyros_robot_menagerie"), "robot", "ur5e.srdf")),
                         packages_path = get_package_share_directory("mujoco_ros_sim"),
                        )

        self._ee_name: Final[str] = "ee_link"

    def compute_pose(self, q: np.ndarray) -> np.ndarray:
        return super().compute_pose(q, self._ee_name)

    def compute_jacobian(self, q: np.ndarray) -> np.ndarray:
        return super().compute_jacobian(q, self._ee_name)

    def compute_jacobian_time_variation(self, q: np.ndarray, qdot: np.ndarray) -> np.ndarray:
        return super().compute_jacobian_time_variation(q, qdot, self._ee_name)

    def compute_velocity(self, q: np.ndarray, qdot: np.ndarray) -> np.ndarray:
        return super().compute_velocity(q, qdot, self._ee_name)

    def compute_manipulability(self, q: np.ndarray, qdot: np.ndarray, with_grad: bool = False, with_graddot: bool = False) -> Tuple[float, np.ndarray, np.ndarray]:
        return super().compute_manipulability(q, qdot, with_grad, with_graddot, self._ee_name)

    def get_pose(self) -> np.ndarray:
        return super().get_pose(self._ee_name)

    def get_jacobian(self) -> np.ndarray:
        return super().get_jacobian(self._ee_name)

    def get_jacobian_dot(self) -> np.ndarray:
        return super().get_jacobian_time_variation(self._ee_name)

    def get_velocity(self) -> np.ndarray:
        return super().get_velocity(self._ee_name)

    def get_manipulability(self, with_grad: bool = False, with_graddot: bool = False):
        return super().get_manipulability(with_grad, with_graddot, self._ee_name)
    
    @property
    def ee_name(self) -> str:
        return self._ee_name