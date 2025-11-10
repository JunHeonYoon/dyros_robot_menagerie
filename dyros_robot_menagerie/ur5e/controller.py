import math
from typing import Dict
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from mujoco_ros_sim import ControllerInterface
from dyros_robot_menagerie.ur5e.robot_data import (UR5eRobotData,
                                                   JOINT_DOF,
                                                   TASK_DOF,
                                                   )
from drc.manipulator.robot_controller import RobotController

"""
MuJoCo Model Information: universal_robots_ur5e
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  0 | shoulder_pan_joint   | Hinge  |  1 |  1 |     0 |    0
  1 | shoulder_lift_joint  | Hinge  |  1 |  1 |     1 |    1
  2 | elbow_joint          | Hinge  |  1 |  1 |     2 |    2
  3 | wrist_1_joint        | Hinge  |  1 |  1 |     3 |    3
  4 | wrist_2_joint        | Hinge  |  1 |  1 |     4 |    4
  5 | wrist_3_joint        | Hinge  |  1 |  1 |     5 |    5

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | shoulder_pan         | Joint   | shoulder_pan_joint
  1 | shoulder_lift        | Joint   | shoulder_lift_joint
  2 | elbow                | Joint   | elbow_joint
  3 | wrist_1              | Joint   | wrist_1_joint
  4 | wrist_2              | Joint   | wrist_2_joint
  5 | wrist_3              | Joint   | wrist_3_joint

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------

 id | name                        | mode     | resolution
----+-----------------------------+----------+------------
"""
class UR5eControllerPy(ControllerInterface):
    def __init__(self, node: Node) -> None:
        super().__init__(node)
        
        self.dt = 0.01

        self.robot_data       = UR5eRobotData()
        self.robot_controller = RobotController(self.dt, self.robot_data)
        
        ns = "ur5e_controller"
        self._key_sub = node.create_subscription(Int32, f"{ns}/mode_input", self._key_cb, 10)
        self._target_pose_sub = node.create_subscription(PoseStamped, f"{ns}/target_pose", self._target_pose_cb, 10)
        
        self._ee_pose_pub = node.create_publisher(PoseStamped, f"{ns}/ee_pose", 10)

        self.mode: str = "HOME"
        self.is_mode_changed = True
        self.is_goal_pose_changed = False

        self.current_time: float = 0.0
        self.control_start_time: float = 0.0

        self.q            = np.zeros(JOINT_DOF)
        self.q_init       = np.zeros(JOINT_DOF)
        self.qdot         = np.zeros(JOINT_DOF)
        self.qdot_desired = np.zeros(JOINT_DOF)
        self.qdot_init    = np.zeros(JOINT_DOF)

        self.x            = np.eye(4)
        self.x_init       = np.eye(4)
        self.x_desired    = np.eye(4)
        self.xdot         = np.zeros(TASK_DOF)
        self.xdot_init    = np.zeros(TASK_DOF)
        self.xdot_desired = np.zeros(TASK_DOF)

        self.x_goal = np.eye(4)

        self.q_desired    = np.zeros(JOINT_DOF)
        
        lines = [
            "\n=================================================================",
            "=================================================================",
            "URDF Joint Information: UR5e",
            self.robot_data.get_verbose().rstrip("\n"),
            "=================================================================",
            "=================================================================",
        ]
        text = "\n".join(lines)
        self.node.get_logger().info("\033[1;34m\n" + text + "\033[0m")


    def starting(self) -> None:
        self._ee_timer = self.node.create_timer(0.01, self._pub_ee_pose_cb)

    def updateState(self,
                    pos_dict: Dict[str, np.ndarray],
                    vel_dict: Dict[str, np.ndarray],
                    tau_ext_dict: Dict[str, np.ndarray],
                    sensor_dict: Dict[str, np.ndarray],
                    current_time: float,
                    ) -> None:
        self.current_time = current_time

        # get manipulator joint
        self.q[0] = pos_dict["shoulder_pan_joint"][0]
        self.q[1] = pos_dict["shoulder_lift_joint"][0]
        self.q[2] = pos_dict["elbow_joint"][0]
        self.q[3] = pos_dict["wrist_1_joint"][0]
        self.q[4] = pos_dict["wrist_2_joint"][0]
        self.q[5] = pos_dict["wrist_3_joint"][0]
        self.qdot[0] = vel_dict["shoulder_pan_joint"][0]
        self.qdot[1] = vel_dict["shoulder_lift_joint"][0]
        self.qdot[2] = vel_dict["elbow_joint"][0]
        self.qdot[3] = vel_dict["wrist_1_joint"][0]
        self.qdot[4] = vel_dict["wrist_2_joint"][0]
        self.qdot[5] = vel_dict["wrist_3_joint"][0]

        if not self.robot_data.update_state(self.q, self.qdot):
            self.node.get_logger().error("[UR5eRobotData] Failed to update robot state.")

        # get ee
        self.x     = self.robot_data.get_pose()
        self.xdot = self.robot_data.get_velocity()
        
    def updateRGBDImage(self, rgbd_dict: Dict[str, Dict[str, np.ndarray]]) -> None:
        pass

    def compute(self) -> None:
        if self.is_mode_changed:
            self.is_mode_changed = False
            self.control_start_time = self.current_time

            self.q_init          = self.q.copy()
            self.qdot_init       = self.qdot.copy()
            self.q_desired       = self.q_init.copy()
            self.qdot_desired[:] = 0.0

            self.x_init          = self.x.copy()
            self.xdot_init       = self.xdot.copy()
            self.x_desired       = self.x_init.copy()
            self.xdot_desired[:] = 0.0
            
            self.x_goal  = self.x_init.copy()

        if self.mode == "HOME":
            target_q = np.array([-math.pi / 2, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0])
            self.q_desired = self.robot_controller.move_joint_position_cubic(q_target     = target_q,
                                                                             qdot_target  = np.zeros(JOINT_DOF),
                                                                             q_init       = self.q_init,
                                                                             qdot_init    = self.qdot_init,
                                                                             current_time = self.current_time,
                                                                             init_time    = self.control_start_time,
                                                                             duration     = 4.0,
                                                                             )

        elif self.mode in ("CLIK", "QPIK"):
            if self.is_goal_pose_changed:
                self.is_goal_pose_changed = False
                self.control_start_time = self.current_time
                
                self.x_init  = self.x.copy()
                self.xdot_init = self.xdot.copy()

            if self.mode == "CLIK":
                self.qdot_desired = self.robot_controller.CLIK_cubic(x_target     = self.x_goal,
                                                                     xdot_target  = np.zeros(TASK_DOF),
                                                                     x_init       = self.x_init,
                                                                     xdot_init    = self.xdot_init,
                                                                     current_time = self.current_time,
                                                                     init_time    = self.control_start_time,
                                                                     duration     = 4.0,
                                                                     link_name    = self.robot_data.ee_name,
                                                                     )

                self.q_desired += self.dt * self.qdot_desired

            elif self.mode == "QPIK":
                self.qdot_desired = self.robot_controller.QPIK_cubic(x_target     = self.x_goal,
                                                                     xdot_target  = np.zeros(TASK_DOF),
                                                                     x_init       = self.x_init,
                                                                     xdot_init    = self.xdot_init,
                                                                     current_time = self.current_time,
                                                                     init_time    = self.control_start_time,
                                                                     duration     = 4.0,
                                                                     link_name    = self.robot_data.ee_name,
                                                                     )
                self.q_desired += self.dt * self.qdot_desired
        else:
            self.q_desired = self.q_init.copy()

    def getCtrlInput(self) -> Dict[str, float]:
        return {"shoulder_pan" : self.q_desired[0],
                "shoulder_lift": self.q_desired[1],
                "elbow"        : self.q_desired[2],
                "wrist_1"      : self.q_desired[3],
                "wrist_2"      : self.q_desired[4],
                "wrist_3"      : self.q_desired[5],
                }

    def _set_mode(self, mode: str) -> None:
        self.mode = mode
        self.is_mode_changed = True
        self.node.get_logger().info(f"Mode changed: {mode}")

    def _key_cb(self, msg: Int32) -> None:
        mapping = {1: "HOME",
                   2: "CLIK",
                   3: "QPIK",
                   }
        self._set_mode(mapping.get(msg.data, "NONE"))

    def _target_pose_cb(self, msg: PoseStamped) -> None:
        self.is_goal_pose_changed = True

        quat = np.array([msg.pose.orientation.x,
                         msg.pose.orientation.y,
                         msg.pose.orientation.z,
                         msg.pose.orientation.w,
                         ])
        rot = R.from_quat(quat).as_matrix()
        self.x_goal = np.eye(4)
        self.x_goal[:3, :3] = rot
        self.x_goal[:3, 3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def _pub_ee_pose_cb(self) -> None:
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.node.get_clock().now().to_msg()

        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.x[:3, 3]
        quat = R.from_matrix(self.x[:3, :3]).as_quat()  # x, y, z, w
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quat
        self._ee_pose_pub.publish(msg)