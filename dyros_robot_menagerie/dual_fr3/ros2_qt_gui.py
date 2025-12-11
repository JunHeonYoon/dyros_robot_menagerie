#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QPushButton,
    QGridLayout, QGroupBox, QVBoxLayout, QHBoxLayout, QSpacerItem, QSizePolicy
)
from PyQt5.QtCore import QTimer


def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def quaternion_to_rpy(x: float, y: float, z: float, w: float):
    # roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    # yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class Ros2QtGui(Node, QWidget):
    def __init__(self, topic_ns: str = 'dual_fr3_controller'):
        Node.__init__(self, 'ros2_qt_gui')
        QWidget.__init__(self)
        self.setWindowTitle('Dual FR3 Controller GUI')
        self.setMinimumWidth(860)   # compact width but two columns
        self.setMaximumWidth(1020)
        self.setContentsMargins(6, 6, 6, 6)

        # Namespace for topics
        self.ns = topic_ns.rstrip('/')

        # Publishers & Subscribers
        self.mode_pub = self.create_publisher(Int32, f'{self.ns}/mode_input', 10)
        self.r_ee_pose_pub = self.create_publisher(PoseStamped, f'{self.ns}/target_r_ee_pose', 10)
        self.l_ee_pose_pub = self.create_publisher(PoseStamped, f'{self.ns}/target_l_ee_pose', 10)

        self.r_ee_pose_sub = self.create_subscription(
            PoseStamped, f'{self.ns}/r_ee_pose', self.r_ee_pose_callback, 10
        )
        self.l_ee_pose_sub = self.create_subscription(
            PoseStamped, f'{self.ns}/l_ee_pose', self.l_ee_pose_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # State holders
        self.current_r_ee = {'X': 0.0, 'Y': 0.0, 'Z': 0.0,
                             'Roll': 0.0, 'Pitch': 0.0, 'Yaw': 0.0}
        self.current_l_ee = {'X': 0.0, 'Y': 0.0, 'Z': 0.0,
                             'Roll': 0.0, 'Pitch': 0.0, 'Yaw': 0.0}

        # Build compact two-column UI that keeps all original widgets
        self._build_ui()

        # ROS spin timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._spin_once)
        self.timer.start(50)  # 20 Hz

    # ---------------------------- UI ----------------------------
    def _mk_small_lineedit(self, read_only=False):
        le = QLineEdit()
        if read_only:
            le.setReadOnly(True)
        le.setFixedWidth(80)
        return le

    def _build_left_controls(self):
        left_col = QVBoxLayout()

        # Mode box
        mode_box = QGroupBox('Mode Input')
        hl = QHBoxLayout()
        self.mode_input = self._mk_small_lineedit()
        btn_mode = QPushButton('Publish Mode')
        btn_mode.clicked.connect(self._on_publish_mode)
        hl.addWidget(QLabel('Mode:'))
        hl.addWidget(self.mode_input)
        hl.addStretch(1)
        hl.addWidget(btn_mode)
        mode_box.setLayout(hl)
        left_col.addWidget(mode_box)

        # -------- Right EE Target box --------
        r_ee_box = QGroupBox('Right Target EE Pose (m / deg)')
        gr = QGridLayout()
        self.r_ee_pose_inputs = {}
        entries = [('ee_x', 'x'), ('ee_y', 'y'), ('ee_z', 'z'),
                   ('ee_roll', 'roll'), ('ee_pitch', 'pitch'), ('ee_yaw', 'yaw')]
        for idx, (key, label) in enumerate(entries):
            r = idx // 3
            c = idx % 3
            gr.addWidget(QLabel(label + ':'), r, c*2)
            le = self._mk_small_lineedit()
            gr.addWidget(le, r, c*2 + 1)
            self.r_ee_pose_inputs[key] = le

        btn_set_r = QPushButton('Set Current Right')
        btn_set_r.clicked.connect(self._on_set_current_right)
        btn_pose_r = QPushButton('Publish Right EE')
        btn_pose_r.clicked.connect(self._on_publish_r_ee_pose)
        gr.addWidget(btn_set_r, 2, 0, 1, 2)
        gr.addWidget(btn_pose_r, 2, 2, 1, 4)
        r_ee_box.setLayout(gr)
        left_col.addWidget(r_ee_box)

        # -------- Left EE Target box --------
        l_ee_box = QGroupBox('Left Target EE Pose (m / deg)')
        gl = QGridLayout()
        self.l_ee_pose_inputs = {}
        for idx, (key, label) in enumerate(entries):
            r = idx // 3
            c = idx % 3
            gl.addWidget(QLabel(label + ':'), r, c*2)
            le = self._mk_small_lineedit()
            gl.addWidget(le, r, c*2 + 1)
            self.l_ee_pose_inputs[key] = le

        btn_set_l = QPushButton('Set Current Left')
        btn_set_l.clicked.connect(self._on_set_current_left)
        btn_pose_l = QPushButton('Publish Left EE')
        btn_pose_l.clicked.connect(self._on_publish_l_ee_pose)
        gl.addWidget(btn_set_l, 2, 0, 1, 2)
        gl.addWidget(btn_pose_l, 2, 2, 1, 4)
        l_ee_box.setLayout(gl)
        left_col.addWidget(l_ee_box)

        left_col.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))
        return left_col

    def _build_right_feedbacks(self):
        right_col = QVBoxLayout()

        # Joint states (7 right joints + 7 left joints)
        joint_box = QGroupBox('Joint States (rad)')
        gj = QGridLayout()
        self.joint_edits = {}

        # Right FR3 joints: r_fr3_joint1 ... r_fr3_joint7
        for i in range(7):
            jname = f'fr3_r_joint{i+1}'
            gj.addWidget(QLabel(jname + ':'), i, 2)
            le = self._mk_small_lineedit(read_only=True)
            gj.addWidget(le, i, 3)
            self.joint_edits[jname] = le

        # Left FR3 joints: l_fr3_joint1 ... l_fr3_joint7
        for i in range(7):
            jname = f'fr3_l_joint{i+1}'
            gj.addWidget(QLabel(jname + ':'), i, 4)
            le = self._mk_small_lineedit(read_only=True)
            gj.addWidget(le, i, 5)
            self.joint_edits[jname] = le

        joint_box.setLayout(gj)
        right_col.addWidget(joint_box)

        # Right EE pose feedback
        r_ee_fb_box = QGroupBox('Current Right EE Pose (m / deg)')
        gr = QGridLayout()
        self.r_ee_edits = {}
        # XYZ
        for i, lab in enumerate(['X', 'Y', 'Z']):
            gr.addWidget(QLabel(lab + ':'), 0, i*2)
            le = self._mk_small_lineedit(read_only=True)
            gr.addWidget(le, 0, i*2 + 1)
            self.r_ee_edits[lab] = le
        # RPY
        for i, lab in enumerate(['Roll', 'Pitch', 'Yaw']):
            gr.addWidget(QLabel(lab + ' (deg):'), 1, i*2)
            le = self._mk_small_lineedit(read_only=True)
            gr.addWidget(le, 1, i*2 + 1)
            self.r_ee_edits[lab] = le
        r_ee_fb_box.setLayout(gr)
        right_col.addWidget(r_ee_fb_box)

        # Left EE pose feedback
        l_ee_fb_box = QGroupBox('Current Left EE Pose (m / deg)')
        gl = QGridLayout()
        self.l_ee_edits = {}
        # XYZ
        for i, lab in enumerate(['X', 'Y', 'Z']):
            gl.addWidget(QLabel(lab + ':'), 0, i*2)
            le = self._mk_small_lineedit(read_only=True)
            gl.addWidget(le, 0, i*2 + 1)
            self.l_ee_edits[lab] = le
        # RPY
        for i, lab in enumerate(['Roll', 'Pitch', 'Yaw']):
            gl.addWidget(QLabel(lab + ' (deg):'), 1, i*2)
            le = self._mk_small_lineedit(read_only=True)
            gl.addWidget(le, 1, i*2 + 1)
            self.l_ee_edits[lab] = le
        l_ee_fb_box.setLayout(gl)
        right_col.addWidget(l_ee_fb_box)

        right_col.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))
        return right_col

    def _build_ui(self):
        root = QHBoxLayout()
        root.setSpacing(10)

        left_col = self._build_left_controls()
        right_col = self._build_right_feedbacks()

        root.addLayout(left_col, 1)
        root.addLayout(right_col, 1)

        self.setLayout(root)

    # ------------------------- ROS Spin -------------------------
    def _spin_once(self):
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except Exception:
            pass

    # ---------------------- Button handlers ---------------------
    def _on_publish_mode(self):
        msg = Int32()
        try:
            msg.data = int(self.mode_input.text())
        except Exception:
            msg.data = 0
        self.mode_pub.publish(msg)

    def _on_publish_r_ee_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        try:
            msg.pose.position.x = float(self.r_ee_pose_inputs['ee_x'].text())
            msg.pose.position.y = float(self.r_ee_pose_inputs['ee_y'].text())
            msg.pose.position.z = float(self.r_ee_pose_inputs['ee_z'].text())
            roll = math.radians(float(self.r_ee_pose_inputs['ee_roll'].text()))
            pitch = math.radians(float(self.r_ee_pose_inputs['ee_pitch'].text()))
            yaw = math.radians(float(self.r_ee_pose_inputs['ee_yaw'].text()))
            qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
        except Exception:
            # leave zeros with valid header
            pass
        self.r_ee_pose_pub.publish(msg)

    def _on_publish_l_ee_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        try:
            msg.pose.position.x = float(self.l_ee_pose_inputs['ee_x'].text())
            msg.pose.position.y = float(self.l_ee_pose_inputs['ee_y'].text())
            msg.pose.position.z = float(self.l_ee_pose_inputs['ee_z'].text())
            roll = math.radians(float(self.l_ee_pose_inputs['ee_roll'].text()))
            pitch = math.radians(float(self.l_ee_pose_inputs['ee_pitch'].text()))
            yaw = math.radians(float(self.l_ee_pose_inputs['ee_yaw'].text()))
            qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
        except Exception:
            # leave zeros with valid header
            pass
        self.l_ee_pose_pub.publish(msg)


    def _on_set_current_right(self):
        """오른팔 EE 현재값으로 오른팔 타겟 입력창 채우기."""
        self.r_ee_pose_inputs['ee_x'].setText(f"{self.current_r_ee.get('X', 0.0):.3f}")
        self.r_ee_pose_inputs['ee_y'].setText(f"{self.current_r_ee.get('Y', 0.0):.3f}")
        self.r_ee_pose_inputs['ee_z'].setText(f"{self.current_r_ee.get('Z', 0.0):.3f}")
        self.r_ee_pose_inputs['ee_roll'].setText(f"{math.degrees(self.current_r_ee.get('Roll', 0.0)):.3f}")
        self.r_ee_pose_inputs['ee_pitch'].setText(f"{math.degrees(self.current_r_ee.get('Pitch', 0.0)):.3f}")
        self.r_ee_pose_inputs['ee_yaw'].setText(f"{math.degrees(self.current_r_ee.get('Yaw', 0.0)):.3f}")

    def _on_set_current_left(self):
        """왼팔 EE 현재값으로 왼팔 타겟 입력창 채우기."""
        self.l_ee_pose_inputs['ee_x'].setText(f"{self.current_l_ee.get('X', 0.0):.3f}")
        self.l_ee_pose_inputs['ee_y'].setText(f"{self.current_l_ee.get('Y', 0.0):.3f}")
        self.l_ee_pose_inputs['ee_z'].setText(f"{self.current_l_ee.get('Z', 0.0):.3f}")
        self.l_ee_pose_inputs['ee_roll'].setText(f"{math.degrees(self.current_l_ee.get('Roll', 0.0)):.3f}")
        self.l_ee_pose_inputs['ee_pitch'].setText(f"{math.degrees(self.current_l_ee.get('Pitch', 0.0)):.3f}")
        self.l_ee_pose_inputs['ee_yaw'].setText(f"{math.degrees(self.current_l_ee.get('Yaw', 0.0)):.3f}")

    # ------------------------ Subscribers -----------------------
    def joint_state_callback(self, msg: JointState):
        # display positions
        n = min(len(msg.name), len(msg.position))
        for name, pos in zip(msg.name[:n], msg.position[:n]):
            if name in self.joint_edits:
                self.joint_edits[name].setText(f"{pos:.3f}")

    def r_ee_pose_callback(self, msg: PoseStamped):
        """오른팔 EE pose 콜백: 상태 저장 + 오른팔 피드백 박스 갱신."""
        x, y, z, w = (msg.pose.orientation.x,
                      msg.pose.orientation.y,
                      msg.pose.orientation.z,
                      msg.pose.orientation.w)
        roll, pitch, yaw = quaternion_to_rpy(x, y, z, w)

        self.current_r_ee.update({
            'X': msg.pose.position.x,
            'Y': msg.pose.position.y,
            'Z': msg.pose.position.z,
            'Roll': roll,
            'Pitch': pitch,
            'Yaw': yaw
        })

        self.r_ee_edits['X'].setText(f"{self.current_r_ee['X']:.3f}")
        self.r_ee_edits['Y'].setText(f"{self.current_r_ee['Y']:.3f}")
        self.r_ee_edits['Z'].setText(f"{self.current_r_ee['Z']:.3f}")
        self.r_ee_edits['Roll'].setText(f"{math.degrees(self.current_r_ee['Roll']):.2f}")
        self.r_ee_edits['Pitch'].setText(f"{math.degrees(self.current_r_ee['Pitch']):.2f}")
        self.r_ee_edits['Yaw'].setText(f"{math.degrees(self.current_r_ee['Yaw']):.2f}")

    def l_ee_pose_callback(self, msg: PoseStamped):
        """왼팔 EE pose 콜백: 상태 저장 + 왼팔 피드백 박스 갱신."""
        x, y, z, w = (msg.pose.orientation.x,
                      msg.pose.orientation.y,
                      msg.pose.orientation.z,
                      msg.pose.orientation.w)
        roll, pitch, yaw = quaternion_to_rpy(x, y, z, w)

        self.current_l_ee.update({
            'X': msg.pose.position.x,
            'Y': msg.pose.position.y,
            'Z': msg.pose.position.z,
            'Roll': roll,
            'Pitch': pitch,
            'Yaw': yaw
        })

        self.l_ee_edits['X'].setText(f"{self.current_l_ee['X']:.3f}")
        self.l_ee_edits['Y'].setText(f"{self.current_l_ee['Y']:.3f}")
        self.l_ee_edits['Z'].setText(f"{self.current_l_ee['Z']:.3f}")
        self.l_ee_edits['Roll'].setText(f"{math.degrees(self.current_l_ee['Roll']):.2f}")
        self.l_ee_edits['Pitch'].setText(f"{math.degrees(self.current_l_ee['Pitch']):.2f}")
        self.l_ee_edits['Yaw'].setText(f"{math.degrees(self.current_l_ee['Yaw']):.2f}")

    # ------------------------ Qt events -------------------------
    def closeEvent(self, event):
        self.timer.stop()
        event.accept()


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = Ros2QtGui(topic_ns='dual_fr3_controller')
    gui.show()
    try:
        exit_code = app.exec_()
    finally:
        gui.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
