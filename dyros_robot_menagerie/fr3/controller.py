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
from drc import TaskSpaceData

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

        self._ee_pose_pub      = node.create_publisher(PoseStamped, f"{ns}/ee_pose", 10)
        self.handeye_rgb_pub   = node.create_publisher(Image, f'{ns}/handeye/rgb/image_raw', qos)
        self.handeye_depth_pub = node.create_publisher(Image, f'{ns}/handeye/depth/image_raw', qos)

        self.bridge_ = CvBridge()

        self.mode: str = "HOME"
        self.is_mode_changed    = True
        self.is_goal_pose_changed = False

        self.current_time: float     = 0.0
        self.control_start_time: float = 0.0

        self.q            = np.zeros(JOINT_DOF)
        self.q_desired    = np.zeros(JOINT_DOF)
        self.q_init       = np.zeros(JOINT_DOF)
        self.qdot         = np.zeros(JOINT_DOF)
        self.qdot_desired = np.zeros(JOINT_DOF)
        self.qdot_init    = np.zeros(JOINT_DOF)

        self.ee_name = "fr3_hand_tcp"
        self.x_goal  = np.eye(4)
        self.link_ee_task: Dict[str, TaskSpaceData] = {
            self.ee_name: TaskSpaceData.Zero()
        }

        self.torque_desired = np.zeros(JOINT_DOF)

        self.handeye_rgb   = None
        self.handeye_depth = None

        self._set_drc_gains()

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
        self._ee_timer            = self.node.create_timer(0.05,  self._pub_ee_pose_cb)
        self.handeye_image_timer  = self.node.create_timer(0.016, self._pub_handeye_image_cb)

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
        self.link_ee_task[self.ee_name].x    = self.robot_data.get_pose(self.ee_name)
        self.link_ee_task[self.ee_name].xdot = self.robot_data.get_velocity(self.ee_name).reshape(-1)

    def updateRGBDImage(self, rgbd_dict: Dict[str, Dict[str, np.ndarray]]) -> None:
        if "hand_eye" in rgbd_dict:
            self.handeye_rgb   = rgbd_dict["hand_eye"].get("rgb")
            self.handeye_depth = rgbd_dict["hand_eye"].get("depth")

    def compute(self) -> None:
        if self.is_mode_changed:
            self.is_mode_changed = False
            self.control_start_time = self.current_time

            self.q_init          = self.q.copy()
            self.qdot_init       = self.qdot.copy()
            self.q_desired       = self.q_init.copy()
            self.qdot_desired[:] = 0.0

            self.x_goal = self.link_ee_task[self.ee_name].x.copy()
            self.link_ee_task[self.ee_name].setInit()
            self.link_ee_task[self.ee_name].setDesired()
            self.link_ee_task[self.ee_name].xdot[:] = 0.0

        if self.mode in ("CLIK", "OSF", "QPIK", "QPID"):
            if self.is_goal_pose_changed:
                self.is_goal_pose_changed = False
                self.control_start_time = self.current_time

                self.link_ee_task[self.ee_name].setInit()
                self.link_ee_task[self.ee_name].xdot[:] = 0.0
                self.link_ee_task[self.ee_name].x_desired = self.x_goal.copy()

        if self.mode == "HOME":
            target_q = np.array([0.0, 0.0, 0.0, -math.pi / 2.0, 0.0, math.pi / 2.0, math.pi / 4.0])
            self.torque_desired = self.robot_controller.move_joint_torque_cubic(
                q_target     = target_q,
                qdot_target  = np.zeros(JOINT_DOF),
                q_init       = self.q_init,
                qdot_init    = self.qdot_init,
                current_time = self.current_time,
                init_time    = self.control_start_time,
                duration     = 4.0,
                use_mass     = False,
            )

        elif self.mode == "CLIK":
            self.qdot_desired = self.robot_controller.CLIK_cubic(
                link_task_data = self.link_ee_task,
                current_time   = self.current_time,
                init_time      = self.control_start_time,
                duration       = 4.0,
            )
            self.q_desired += self.dt * self.qdot_desired
            self.torque_desired = self.robot_controller.move_joint_torque_step(
                q_target    = self.q_desired,
                qdot_target = self.qdot_desired,
                use_mass    = False,
            )

        elif self.mode == "QPIK":
            ok, self.qdot_desired = self.robot_controller.QPIK_cubic(
                link_task_data = self.link_ee_task,
                current_time   = self.current_time,
                init_time      = self.control_start_time,
                duration       = 4.0,
            )
            if not ok:
                self.qdot_desired = np.zeros(JOINT_DOF)
            self.q_desired += self.dt * self.qdot_desired
            self.torque_desired = self.robot_controller.move_joint_torque_step(
                q_target    = self.q_desired,
                qdot_target = self.qdot_desired,
                use_mass    = False,
            )

        elif self.mode == "OSF":
            null_torque = self.robot_controller.move_joint_torque_step(
                q_target    = self.q_init,
                qdot_target = np.zeros(JOINT_DOF),
                use_mass    = False,
            )
            self.torque_desired = self.robot_controller.OSF_cubic(
                link_task_data = self.link_ee_task,
                current_time   = self.current_time,
                init_time      = self.control_start_time,
                duration       = 4.0,
                null_torque    = null_torque,
            )

        elif self.mode == "QPID":
            ok, self.torque_desired = self.robot_controller.QPID_cubic(
                link_task_data = self.link_ee_task,
                current_time   = self.current_time,
                init_time      = self.control_start_time,
                duration       = 4.0,
            )
            if not ok:
                self.torque_desired = self.robot_data.get_gravity()

        elif self.mode == "Gravity_compensattion_W_QPID":
            self.link_ee_task[self.ee_name].xddot_desired[:] = 0.0
            ok, self.torque_desired = self.robot_controller.QPID(self.link_ee_task)
            if not ok:
                self.torque_desired = self.robot_data.get_gravity()

        else:
            self.torque_desired = self.robot_data.get_gravity()

    def getCtrlInput(self) -> Dict[str, float]:
        return {f"fr3_joint{i + 1}": float(self.torque_desired[i]) for i in range(JOINT_DOF)}

    def _set_drc_gains(self) -> bool:
        joint = "dyros_robot_controller.manipulator_joint_gains."
        task  = "dyros_robot_controller.task_gains."
        qpik  = "dyros_robot_controller.QPIK_weight."
        qpid  = "dyros_robot_controller.QPID_weight."

        joint_kp = self.node.declare_parameter(joint + "kp", [1.0] * JOINT_DOF).value
        joint_kv = self.node.declare_parameter(joint + "kv", [1.0] * JOINT_DOF).value

        task_ik_kp = self.node.declare_parameter(task + "ik.kp", [1.0] * TASK_DOF).value
        task_id_kp = self.node.declare_parameter(task + "id.kp", [1.0] * TASK_DOF).value
        task_id_kv = self.node.declare_parameter(task + "id.kd", [1.0] * TASK_DOF).value

        qpik_tracking    = self.node.declare_parameter(qpik + "tracking.weights",               [1.0] * TASK_DOF).value
        qpik_vel_damping = self.node.declare_parameter(qpik + "joint.velocity.manipulator",     [1.0] * JOINT_DOF).value
        qpik_acc_damping = self.node.declare_parameter(qpik + "joint.acceleration.manipulator", [1.0] * JOINT_DOF).value

        qpid_tracking    = self.node.declare_parameter(qpid + "tracking.weights",               [1.0] * TASK_DOF).value
        qpid_vel_damping = self.node.declare_parameter(qpid + "joint.velocity.manipulator",     [1.0] * JOINT_DOF).value
        qpid_acc_damping = self.node.declare_parameter(qpid + "joint.acceleration.manipulator", [1.0] * JOINT_DOF).value

        def check(name: str, expected: int, got) -> bool:
            if len(got) != expected:
                self.node.get_logger().error(f"Parameter '{name}' expected {expected} values, got {len(got)}.")
                return False
            return True

        if not check(joint + "kp", JOINT_DOF, joint_kp): return False
        if not check(joint + "kv", JOINT_DOF, joint_kv): return False
        if not check(task + "ik.kp", TASK_DOF, task_ik_kp): return False
        if not check(task + "id.kp", TASK_DOF, task_id_kp): return False
        if not check(task + "id.kd", TASK_DOF, task_id_kv): return False
        if not check(qpik + "tracking.weights",               TASK_DOF,  qpik_tracking):    return False
        if not check(qpik + "joint.velocity.manipulator",     JOINT_DOF, qpik_vel_damping): return False
        if not check(qpik + "joint.acceleration.manipulator", JOINT_DOF, qpik_acc_damping): return False
        if not check(qpid + "tracking.weights",               TASK_DOF,  qpid_tracking):    return False
        if not check(qpid + "joint.velocity.manipulator",     JOINT_DOF, qpid_vel_damping): return False
        if not check(qpid + "joint.acceleration.manipulator", JOINT_DOF, qpid_acc_damping): return False

        self.robot_controller.set_joint_gain(np.array(joint_kp), np.array(joint_kv))
        self.robot_controller.set_IK_gain(np.array(task_ik_kp))
        self.robot_controller.set_ID_gain(np.array(task_id_kp), np.array(task_id_kv))
        self.robot_controller.set_QPIK_gain(np.array(qpik_tracking), np.array(qpik_vel_damping), np.array(qpik_acc_damping))
        self.robot_controller.set_QPID_gain(np.array(qpid_tracking), np.array(qpid_vel_damping), np.array(qpid_acc_damping))

        msg = (
            "dyros robot controller gains\n"
            "\tmanipulator_joint_gains\n"
            f"\t\tKp: {np.array(joint_kp)}\n"
            f"\t\tKv: {np.array(joint_kv)}\n"
            "\ttask_gains\n"
            f"\t\tik.Kp: {np.array(task_ik_kp)}\n"
            f"\t\tid.Kp: {np.array(task_id_kp)}\n"
            f"\t\tid.Kv: {np.array(task_id_kv)}\n"
            "\tQPIK_weight\n"
            f"\t\ttracking.weights: {np.array(qpik_tracking)}\n"
            f"\t\tjoint.velocity.manipulator: {np.array(qpik_vel_damping)}\n"
            f"\t\tjoint.acceleration.manipulator: {np.array(qpik_acc_damping)}\n"
            "\tQPID_weight\n"
            f"\t\ttracking.weights: {np.array(qpid_tracking)}\n"
            f"\t\tjoint.velocity.manipulator: {np.array(qpid_vel_damping)}\n"
            f"\t\tjoint.acceleration.manipulator: {np.array(qpid_acc_damping)}"
        )
        self.node.get_logger().info("\033[1;34m" + msg + "\033[0m")
        return True

    def _set_mode(self, mode: str) -> None:
        self.mode = mode
        self.is_mode_changed = True
        self.node.get_logger().info(f"Mode changed: {mode}")

    def _key_cb(self, msg: Int32) -> None:
        mapping = {
            1: "HOME",
            2: "CLIK",
            3: "QPIK",
            4: "OSF",
            5: "QPID",
            6: "Gravity_compensattion_W_QPID",
        }
        self._set_mode(mapping.get(msg.data, "NONE"))

    def _target_pose_cb(self, msg: PoseStamped) -> None:
        self.node.get_logger().info(
            f"Target pose received: position=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}), "
            f"orientation=({msg.pose.orientation.x:.3f}, {msg.pose.orientation.y:.3f}, "
            f"{msg.pose.orientation.z:.3f}, {msg.pose.orientation.w:.3f})"
        )
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

        x = self.link_ee_task[self.ee_name].x
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x[:3, 3]
        quat = R.from_matrix(x[:3, :3]).as_quat()  # x, y, z, w
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quat
        self._ee_pose_pub.publish(msg)

    def _pub_handeye_image_cb(self):
        if self.handeye_rgb is None or self.handeye_depth is None:
            self.node.get_logger().warn("handeye_image not ready yet")
            return

        img_bgr   = np.ascontiguousarray(self.handeye_rgb)[:, :, ::-1]
        img_depth = np.ascontiguousarray(self.handeye_depth)

        if img_depth.ndim == 3 and img_depth.shape[2] == 1:
            img_depth = img_depth[..., 0]

        if img_depth.dtype == np.uint16:
            img_depth = img_depth.astype(np.float32) / 1000.0
        elif img_depth.dtype in (np.float32, np.float64):
            img_depth = img_depth.astype(np.float32)
        else:
            img_depth = img_depth.astype(np.float32)

        rgb_image   = self.bridge_.cv2_to_imgmsg(img_bgr,   encoding='bgr8')
        depth_image = self.bridge_.cv2_to_imgmsg(img_depth, encoding='32FC1')

        rgb_image.header.stamp       = self.node.get_clock().now().to_msg()
        rgb_image.header.frame_id    = 'hand_eye_cam_frame'
        depth_image.header.stamp     = self.node.get_clock().now().to_msg()
        depth_image.header.frame_id  = 'hand_eye_cam_frame'
        self.handeye_rgb_pub.publish(rgb_image)
        self.handeye_depth_pub.publish(depth_image)
