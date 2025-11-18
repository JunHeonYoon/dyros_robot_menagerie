#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Twist
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
    def __init__(self, topic_ns: str = 'fr3_xls_controller'):
        Node.__init__(self, 'ros2_qt_gui')
        QWidget.__init__(self)
        self.setWindowTitle('FR3-XLS Controller GUI')
        self.setMinimumWidth(760)
        self.setMaximumWidth(920)
        self.setContentsMargins(6, 6, 6, 6)

        # Namespace for topics
        self.ns = topic_ns.rstrip('/')

        # Publishers & Subscribers
        self.mode_pub = self.create_publisher(Int32, f'{self.ns}/mode_input', 10)
        self.ee_pose_pub = self.create_publisher(PoseStamped, f'{self.ns}/target_ee_pose', 10)
        self.base_vel_pub = self.create_publisher(Twist, f'{self.ns}/cmd_vel', 10)

        self.ee_pose_sub = self.create_subscription(PoseStamped, f'{self.ns}/ee_pose', self.ee_pose_callback, 10)
        self.base_vel_sub = self.create_subscription(Twist, f'{self.ns}/base_vel', self.base_vel_callback, 10)
        self.base_pose_sub = self.create_subscription(PoseStamped, f'{self.ns}/base_pose', self.base_pose_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # State holders
        self.current_ee = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'Roll': 0.0, 'Pitch': 0.0, 'Yaw': 0.0}
        self.current_base_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0} 
        self.current_base_vel = {'vx': 0.0, 'vy': 0.0, 'w': 0.0}

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

        # EE Target box
        ee_box = QGroupBox('Target EE Pose (m / deg)')
        g = QGridLayout()
        self.ee_pose_inputs = {}
        entries = [('ee_x', 'x'), ('ee_y', 'y'), ('ee_z', 'z'),
                   ('ee_roll', 'roll'), ('ee_pitch', 'pitch'), ('ee_yaw', 'yaw')]
        for idx, (key, label) in enumerate(entries):
            r = idx // 3
            c = idx % 3
            g.addWidget(QLabel(label + ':'), r, c*2)
            le = self._mk_small_lineedit()
            g.addWidget(le, r, c*2 + 1)
            self.ee_pose_inputs[key] = le

        btn_set = QPushButton('Set Current')
        btn_set.clicked.connect(self._on_set_current)
        btn_pose = QPushButton('Publish EE Pose')
        btn_pose.clicked.connect(self._on_publish_ee_pose)
        g.addWidget(btn_set, 2, 0, 1, 2)
        g.addWidget(btn_pose, 2, 2, 1, 4)
        ee_box.setLayout(g)
        left_col.addWidget(ee_box)

        # Base Velocity box
        vel_box = QGroupBox('Target Base Velocity')
        hv = QHBoxLayout()
        self.base_vel_inputs = {'b_v': self._mk_small_lineedit(),
                                'b_w': self._mk_small_lineedit()}
        hv.addWidget(QLabel('v (m/s):'))
        hv.addWidget(self.base_vel_inputs['b_v'])
        hv.addSpacing(12)
        hv.addWidget(QLabel('w (rad/s):'))
        hv.addWidget(self.base_vel_inputs['b_w'])
        hv.addStretch(1)
        btn_vel = QPushButton('Publish Velocity')
        btn_vel.clicked.connect(self._on_publish_base_vel)
        hv.addWidget(btn_vel)
        vel_box.setLayout(hv)
        left_col.addWidget(vel_box)

        left_col.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))
        return left_col

    def _build_right_feedbacks(self):
        right_col = QVBoxLayout()

        # Joint states (wheels + 7 joints)
        joint_box = QGroupBox('Joint States (rad)')
        gj = QGridLayout()
        self.joint_edits = {}
        wheel_names = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']
        for i, jname in enumerate(wheel_names):
            gj.addWidget(QLabel(jname + ':'), i, 0)
            le = self._mk_small_lineedit(read_only=True)
            gj.addWidget(le, i, 1)
            self.joint_edits[jname] = le
        # FR3 joints
        for i in range(7):
            jname = f'fr3_joint{i+1}'
            gj.addWidget(QLabel(jname + ':'), i, 2)
            le = self._mk_small_lineedit(read_only=True)
            gj.addWidget(le, i, 3)
            self.joint_edits[jname] = le
        joint_box.setLayout(gj)
        right_col.addWidget(joint_box)

        # Base pose (x, y, theta)
        base_pose_box = QGroupBox('Base Pose (x, y, theta)')
        gbp = QGridLayout()
        self.base_pose_edits = {}
        for i, lab in enumerate(['x', 'y', 'theta']):
            gbp.addWidget(QLabel(lab + ':'), 0, i*2)
            le = self._mk_small_lineedit(read_only=True)
            gbp.addWidget(le, 0, i*2 + 1)
            self.base_pose_edits[lab] = le
        base_pose_box.setLayout(gbp)
        right_col.addWidget(base_pose_box)

        # Base velocity (vx, vy, w)
        base_vel_box = QGroupBox('Base Velocity (vx, vy, w)')
        gbv = QGridLayout()
        self.base_vel_edits = {}
        for i, lab in enumerate(['vx', 'vy', 'w']):
            gbv.addWidget(QLabel(lab + ':'), 0, i*2)
            le = self._mk_small_lineedit(read_only=True)
            gbv.addWidget(le, 0, i*2 + 1)
            self.base_vel_edits[lab] = le
        base_vel_box.setLayout(gbv)
        right_col.addWidget(base_vel_box)

        # EE pose feedback
        ee_fb_box = QGroupBox('Current EE Pose (m / deg)')
        gef = QGridLayout()
        self.ee_edits = {}
        # XYZ
        for i, lab in enumerate(['X', 'Y', 'Z']):
            gef.addWidget(QLabel(lab + ':'), 0, i*2)
            le = self._mk_small_lineedit(read_only=True)
            gef.addWidget(le, 0, i*2 + 1)
            self.ee_edits[lab] = le
        # RPY in degrees
        for i, lab in enumerate(['Roll', 'Pitch', 'Yaw']):
            gef.addWidget(QLabel(lab + ' (deg):'), 1, i*2)
            le = self._mk_small_lineedit(read_only=True)
            gef.addWidget(le, 1, i*2 + 1)
            self.ee_edits[lab] = le
        ee_fb_box.setLayout(gef)
        right_col.addWidget(ee_fb_box)

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

    def _on_publish_ee_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        try:
            msg.pose.position.x = float(self.ee_pose_inputs['ee_x'].text())
            msg.pose.position.y = float(self.ee_pose_inputs['ee_y'].text())
            msg.pose.position.z = float(self.ee_pose_inputs['ee_z'].text())
            roll = math.radians(float(self.ee_pose_inputs['ee_roll'].text()))
            pitch = math.radians(float(self.ee_pose_inputs['ee_pitch'].text()))
            yaw = math.radians(float(self.ee_pose_inputs['ee_yaw'].text()))
            qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
        except Exception:
            # leave zeros with valid header
            pass
        self.ee_pose_pub.publish(msg)

    def _on_publish_base_vel(self):
        msg = Twist()
        try:
            msg.linear.x = float(self.base_vel_inputs['b_v'].text())
            msg.angular.z = float(self.base_vel_inputs['b_w'].text())
        except Exception:
            pass
        self.base_vel_pub.publish(msg)

    def _on_set_current(self):
        # Fill target fields with current EE pose
        self.ee_pose_inputs['ee_x'].setText(f"{self.current_ee.get('X', 0.0):.3f}")
        self.ee_pose_inputs['ee_y'].setText(f"{self.current_ee.get('Y', 0.0):.3f}")
        self.ee_pose_inputs['ee_z'].setText(f"{self.current_ee.get('Z', 0.0):.3f}")
        self.ee_pose_inputs['ee_roll'].setText(f"{math.degrees(self.current_ee.get('Roll', 0.0)):.3f}")
        self.ee_pose_inputs['ee_pitch'].setText(f"{math.degrees(self.current_ee.get('Pitch', 0.0)):.3f}")
        self.ee_pose_inputs['ee_yaw'].setText(f"{math.degrees(self.current_ee.get('Yaw', 0.0)):.3f}")

    # ------------------------ Subscribers -----------------------
    def joint_state_callback(self, msg: JointState):
        # display positions
        n = min(len(msg.name), len(msg.position))
        for name, pos in zip(msg.name[:n], msg.position[:n]):
            if name in self.joint_edits:
                self.joint_edits[name].setText(f"{pos:.3f}")

    def ee_pose_callback(self, msg: PoseStamped):
        x, y, z, w = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        roll, pitch, yaw = quaternion_to_rpy(x, y, z, w)

        self.current_ee.update({
            'X': msg.pose.position.x,
            'Y': msg.pose.position.y,
            'Z': msg.pose.position.z,
            'Roll': roll,
            'Pitch': pitch,
            'Yaw': yaw
        })

        self.ee_edits['X'].setText(f"{self.current_ee['X']:.3f}")
        self.ee_edits['Y'].setText(f"{self.current_ee['Y']:.3f}")
        self.ee_edits['Z'].setText(f"{self.current_ee['Z']:.3f}")
        self.ee_edits['Roll'].setText(f"{math.degrees(self.current_ee['Roll']):.2f}")
        self.ee_edits['Pitch'].setText(f"{math.degrees(self.current_ee['Pitch']):.2f}")
        self.ee_edits['Yaw'].setText(f"{math.degrees(self.current_ee['Yaw']):.2f}")

    def base_pose_callback(self, msg: PoseStamped):
        x, y, z, w = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        roll, pitch, yaw = quaternion_to_rpy(x, y, z, w)

        self.current_base_pose.update({
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'Z': msg.pose.position.z,
            'theta': yaw
        })

        self.base_pose_edits['x'].setText(f"{self.current_base_pose['x']:.3f}")
        self.base_pose_edits['y'].setText(f"{self.current_base_pose['y']:.3f}")
        self.base_pose_edits['theta'].setText(f"{math.degrees(self.current_base_pose['theta']):.2f}")

    def base_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = getattr(msg.linear, 'y', 0.0)
        w = msg.angular.z
        self.current_base_vel.update({'vx': vx, 'vy': vy, 'w': w})
        self.base_vel_edits['vx'].setText(f"{vx:.3f}")
        self.base_vel_edits['vy'].setText(f"{vy:.3f}")
        self.base_vel_edits['w'].setText(f"{w:.3f}")

    # ------------------------ Qt events -------------------------
    def closeEvent(self, event):
        self.timer.stop()
        event.accept()


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = Ros2QtGui(topic_ns='fr3_xls_controller')
    gui.show()
    try:
        exit_code = app.exec_()
    finally:
        gui.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()