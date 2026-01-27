import math
from typing import Dict
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from mujoco_ros_sim import ControllerInterface
from dyros_robot_menagerie.fr3.robot_data import (FR3RobotData,
                                                  JOINT_DOF,
                                                  TASK_DOF,
                                                  )
from drc.manipulator.robot_controller import RobotController

"""
MuJoCo Model Information: franka_fr3_torque
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  0 | fr3_joint1           | Hinge  |  1 |  1 |     0 |    0
  1 | fr3_joint2           | Hinge  |  1 |  1 |     1 |    1
  2 | fr3_joint3           | Hinge  |  1 |  1 |     2 |    2
  3 | fr3_joint4           | Hinge  |  1 |  1 |     3 |    3
  4 | fr3_joint5           | Hinge  |  1 |  1 |     4 |    4
  5 | fr3_joint6           | Hinge  |  1 |  1 |     5 |    5
  6 | fr3_joint7           | Hinge  |  1 |  1 |     6 |    6

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | fr3_joint1           | Joint   | fr3_joint1
  1 | fr3_joint2           | Joint   | fr3_joint2
  2 | fr3_joint3           | Joint   | fr3_joint3
  3 | fr3_joint4           | Joint   | fr3_joint4
  4 | fr3_joint5           | Joint   | fr3_joint5
  5 | fr3_joint6           | Joint   | fr3_joint6
  6 | fr3_joint7           | Joint   | fr3_joint7

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
  0 | fr3_ee_force                | Force            |   3 |   0 | Site:attachment_site
  1 | fr3_ee_torque               | Torque           |   3 |   3 | Site:attachment_site

 id | name                        | mode     | resolution
----+-----------------------------+----------+------------
  0 | hand_eye                    | -        | 1920x1080
"""
class FR3ControllerPy(ControllerInterface):
    def __init__(self, node: Node) -> None:
        super().__init__(node)
        
        self.dt = 0.001

        self.robot_data       = FR3RobotData(self.dt)
        self.dt = self.robot_data.get_dt()
        self.robot_controller = RobotController(self.robot_data)
        
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        
        ns = "fr3_controller"
        self._key_sub = node.create_subscription(Int32, f"{ns}/mode_input", self._key_cb, 10)
        self._target_pose_sub = node.create_subscription(PoseStamped, f"{ns}/target_pose", self._target_pose_cb, 10)
        
        self._ee_pose_pub = node.create_publisher(PoseStamped, f"{ns}/ee_pose", 10)
        self.handeye_rgb_pub = node.create_publisher(Image, f'{ns}/handeye/rgb/image_raw', qos)
        self.handeye_depth_pub = node.create_publisher(Image, f'{ns}/handeye/depth/image_raw', qos)

        self.bridge_ = CvBridge()

        self.mode: str = "HOME"
        self.is_mode_changed = True
        self.is_goal_pose_changed = False

        self.current_time: float = 0.0
        self.control_start_time: float = 0.0

        self.q            = np.zeros(JOINT_DOF)
        self.q_desired    = np.zeros(JOINT_DOF)
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

        self.torque_desired = np.zeros(JOINT_DOF)
        
        self.handeye_rgb = None
        self.handeye_depth = None
        
        
        lines = [
            "\n=================================================================",
            "=================================================================",
            "URDF Joint Information: FR3",
            self.robot_data.get_verbose().rstrip("\n"),
            "=================================================================",
            "=================================================================",
        ]
        text = "\n".join(lines)
        self.node.get_logger().info("\033[1;34m\n" + text + "\033[0m")


    def starting(self) -> None:
        self._ee_timer = self.node.create_timer(0.01, self._pub_ee_pose_cb)
        self.handeye_image_timer = self.node.create_timer(0.033, self._pub_handeye_image_cb)

    def updateState(self,
                    pos_dict: Dict[str, np.ndarray],
                    vel_dict: Dict[str, np.ndarray],
                    tau_ext_dict: Dict[str, np.ndarray],
                    sensor_dict: Dict[str, np.ndarray],
                    current_time: float,
                    ) -> None:
        self.current_time = current_time

        # get manipulator joint
        for i in range(JOINT_DOF):
            name = f"fr3_joint{i + 1}"
            self.q[i]    = pos_dict[name][0]
            self.qdot[i] = vel_dict[name][0]

        if not self.robot_data.update_state(self.q, self.qdot):
            self.node.get_logger().error("[FR3RobotData] Failed to update robot state.")

        # get ee
        self.x     = self.robot_data.get_pose()
        self.xdot = self.robot_data.get_velocity()
        
    def updateRGBDImage(self, rgbd_dict: Dict[str, Dict[str, np.ndarray]]) -> None:
        if "hand_eye" in rgbd_dict:
            self.handeye_rgb = rgbd_dict.get("hand_eye", {}).get("rgb")
            self.handeye_depth = rgbd_dict.get("hand_eye", {}).get("depth")

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
            target_q = np.array([0.0, 0.0, 0.0, -math.pi / 2.0, 0.0, math.pi / 2.0, math.pi / 4.0])
            self.torque_desired = self.robot_controller.move_joint_torque_cubic(q_target     = target_q,
                                                                                qdot_target  = np.zeros(JOINT_DOF),
                                                                                q_init       = self.q_init,
                                                                                qdot_init    = self.qdot_init,
                                                                                current_time = self.current_time,
                                                                                init_time    = self.control_start_time,
                                                                                duration     = 4.0,
                                                                                )

        elif self.mode in ("CLIK", "QPIK", "OSF", "QPID"):
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
                self.torque_desired = self.robot_controller.move_joint_torque_step(self.q_desired, 
                                                                                   self.qdot_desired)

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
                self.torque_desired = self.robot_controller.move_joint_torque_step(self.q_desired, 
                                                                                   self.qdot_desired)

            elif self.mode == "OSF":
                null_torque = self.robot_controller.move_joint_torque_step(self.q_init, 
                                                                           np.zeros(JOINT_DOF))
                self.torque_desired = self.robot_controller.OSF_cubic(x_target     = self.x_goal,
                                                                      xdot_target  = np.zeros(TASK_DOF),
                                                                      x_init       = self.x_init,
                                                                      xdot_init    = self.xdot_init,
                                                                      current_time = self.current_time,
                                                                      init_time    = self.control_start_time,
                                                                      duration     = 4.0,
                                                                      link_name    = self.robot_data.ee_name,
                                                                      null_torque  = null_torque
                                                                      )

            elif self.mode == "QPID":
                self.torque_desired = self.robot_controller.QPID_cubic(x_target     = self.x_goal,
                                                                      xdot_target  = np.zeros(TASK_DOF),
                                                                      x_init       = self.x_init,
                                                                      xdot_init    = self.xdot_init,
                                                                      current_time = self.current_time,
                                                                      init_time    = self.control_start_time,
                                                                      duration     = 4.0,
                                                                      link_name    = self.robot_data.ee_name,
                                                                      )

        else:
            self.torque_desired = self.robot_data.get_gravity()

    def getCtrlInput(self) -> Dict[str, float]:
        return {f"fr3_joint{i + 1}": float(self.torque_desired[i]) for i in range(JOINT_DOF)}

    def _set_mode(self, mode: str) -> None:
        self.mode = mode
        self.is_mode_changed = True
        self.node.get_logger().info(f"Mode changed: {mode}")

    def _key_cb(self, msg: Int32) -> None:
        mapping = {1: "HOME",
                   2: "CLIK",
                   3: "QPIK",
                   4: "OSF",
                   5: "QPID",
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
        
    def _pub_handeye_image_cb(self):
        if self.handeye_rgb is None or self.handeye_depth is None:
            self.node.get_logger().warn("handeye_image not ready yet")
            return

        img_bgr = np.ascontiguousarray(self.handeye_rgb)[:, :, ::-1]
        img_depth = np.ascontiguousarray(self.handeye_depth)

        if img_depth.ndim == 3 and img_depth.shape[2] == 1:
            img_depth = img_depth[..., 0]

        if img_depth.dtype == np.uint16:
            img_depth = (img_depth.astype(np.float32)) / 1000.0
        elif img_depth.dtype in (np.float32, np.float64):
            img_depth = img_depth.astype(np.float32)
        else:
            img_depth = img_depth.astype(np.float32)
            
        rgb_image = self.bridge_.cv2_to_imgmsg(img_bgr, encoding='bgr8')
        depth_image = self.bridge_.cv2_to_imgmsg(img_depth, encoding='32FC1')
        
        rgb_image.header.stamp = self.node.get_clock().now().to_msg()
        rgb_image.header.frame_id = 'handeye'
        depth_image.header.stamp = self.node.get_clock().now().to_msg()
        depth_image.header.frame_id = 'handeye'
        self.handeye_rgb_pub.publish(rgb_image)
        self.handeye_depth_pub.publish(depth_image)
