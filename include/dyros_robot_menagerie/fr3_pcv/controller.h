#pragma once
#include <pluginlib/class_list_macros.hpp>
#include "mujoco_ros_sim/controller_interface.hpp"

#include "dyros_robot_menagerie/fr3_pcv/robot_data.h"

#include "dyros_robot_controller/mobile_manipulator/robot_controller.h"

#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

// #include "image_io.h"
#include "math_type_define.h"

#include <mutex> 
#include <algorithm>

namespace FR3PCV
{
/*
MuJoCo Model Information: fr3_dyros_pcv
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  0 | -                    | Free   |  7 |  6 |     0 |    0
  1 | front_left_steer     | Hinge  |  1 |  1 |     7 |    6
  2 | front_left_rotate    | Hinge  |  1 |  1 |     8 |    7
  3 | rear_left_steer      | Hinge  |  1 |  1 |     9 |    8
  4 | rear_left_rotate     | Hinge  |  1 |  1 |    10 |    9
  5 | rear_right_steer     | Hinge  |  1 |  1 |    11 |   10
  6 | rear_right_rotate    | Hinge  |  1 |  1 |    12 |   11
  7 | front_right_steer    | Hinge  |  1 |  1 |    13 |   12
  8 | front_right_rotate   | Hinge  |  1 |  1 |    14 |   13
  9 | fr3_joint1           | Hinge  |  1 |  1 |    15 |   14
 10 | fr3_joint2           | Hinge  |  1 |  1 |    16 |   15
 11 | fr3_joint3           | Hinge  |  1 |  1 |    17 |   16
 12 | fr3_joint4           | Hinge  |  1 |  1 |    18 |   17
 13 | fr3_joint5           | Hinge  |  1 |  1 |    19 |   18
 14 | fr3_joint6           | Hinge  |  1 |  1 |    20 |   19
 15 | fr3_joint7           | Hinge  |  1 |  1 |    21 |   20
 16 | finger_joint1        | Slide  |  1 |  1 |    22 |   21
 17 | finger_joint2        | Slide  |  1 |  1 |    23 |   22

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | front_left_rotate    | Joint   | front_left_rotate
  1 | front_right_rotate   | Joint   | front_right_rotate
  2 | rear_left_rotate     | Joint   | rear_left_rotate
  3 | rear_right_rotate    | Joint   | rear_right_rotate
  4 | front_left_steer     | Joint   | front_left_steer
  5 | front_right_steer    | Joint   | front_right_steer
  6 | rear_left_steer      | Joint   | rear_left_steer
  7 | rear_right_steer     | Joint   | rear_right_steer
  8 | fr3_joint1           | Joint   | fr3_joint1
  9 | fr3_joint2           | Joint   | fr3_joint2
 10 | fr3_joint3           | Joint   | fr3_joint3
 11 | fr3_joint4           | Joint   | fr3_joint4
 12 | fr3_joint5           | Joint   | fr3_joint5
 13 | fr3_joint6           | Joint   | fr3_joint6
 14 | fr3_joint7           | Joint   | fr3_joint7
 15 | fr3_hand             | Tendon  | -

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
  0 | position_sensor             | FramePos         |   3 |   0 | Site:dyros_pcv_site
  1 | orientation_sensor          | FrameQuat        |   4 |   3 | Site:dyros_pcv_site
  2 | linear_velocity_sensor      | FrameLinVel      |   3 |   7 | Site:dyros_pcv_site
  3 | angular_velocity_sensor     | FrameAngVel      |   3 |  10 | Site:dyros_pcv_site

 id | name                        | mode     | resolution
----+-----------------------------+----------+------------
  0 | d435_rgb                    | -        | 640x480
*/
    class FR3PCVController final : public MujocoRosSim::ControllerInterface
    {
        public:
            FR3PCVController() = default;
            // ====================================================================================
            // ================================== Core Functions ================================== 
            // ====================================================================================
            void configure(const rclcpp::Node::SharedPtr& node) override;
            void starting() override;
            void updateState(const MujocoRosSim::VecMap&, const MujocoRosSim::VecMap&, const MujocoRosSim::VecMap&, const MujocoRosSim::VecMap&, double) override;
            void updateRGBDImage(const MujocoRosSim::ImageCVMap& images) override;
            void compute() override;
            MujocoRosSim::CtrlInputMap getCtrlInput() const override;

        private:
            // ====================================================================================
            // ===================== Helper / CB / Background Thread Functions ==================== 
            // ====================================================================================
            void setMode(const std::string& mode);
            void keyCallback(const std_msgs::msg::Int32::SharedPtr);
            void subtargetEEPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);
            void subtargetBaseVelCallback(const geometry_msgs::msg::Twist::SharedPtr);
            void subJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr);
            void pubEEPoseCallback();
            void pubBasePoseCallback();
            void pubBaseVelCallback();

            std::shared_ptr<FR3PCV::FR3PCVRobotData> robot_data_;
            std::unique_ptr<drc::MobileManipulator::RobotController> robot_controller_;

            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            key_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_ee_pose_sub_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       target_base_vel_sub_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr    joint_sub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    current_ee_pose_pub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    current_base_pose_pub_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          current_base_vel_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr       joint_pub_;

            rclcpp::TimerBase::SharedPtr current_ee_pose_pub_timer_;
            rclcpp::TimerBase::SharedPtr current_base_pose_pub_timer_;
            rclcpp::TimerBase::SharedPtr current_base_vel_pub_timer_;

            bool is_mode_changed_{true};
            bool is_goal_pose_changed_{false};

            std::string mode_{"HOME"};
            
            double control_start_time_;
            double current_time_;

            //// mobile base
            Vector3d base_vel_; // [lin_x, lin_y, ang] wrt base frame
            Vector3d base_vel_desired_;
            Vector3d base_vel_init_;
            
            //// joint space state
            VirtualVec q_virtual_;
            VirtualVec q_virtual_desired_;
            VirtualVec q_virtual_init_;
            VirtualVec qdot_virtual_;
            VirtualVec qdot_virtual_desired_;
            VirtualVec qdot_virtual_init_;

            ManiVec q_mani_;
            ManiVec q_mani_desired_;
            ManiVec q_mani_init_;
            ManiVec qdot_mani_;
            ManiVec qdot_mani_desired_;
            ManiVec qdot_mani_init_;

            MobiVec q_mobile_;
            MobiVec q_mobile_desired_;
            MobiVec q_mobile_init_;
            MobiVec qdot_mobile_;
            MobiVec qdot_mobile_init_;

            //// operation space state
            std::string link_ee_name_;
            Affine3d x_goal_;
            std::map<std::string, drc::TaskSpaceData> link_ee_task_;

            //// control input
            ManiVec torque_mani_desired_;
            MobiVec qdot_mobile_desired_;

            //// gains
            ManiVec      mani_joint_kp_;
            ManiVec      mani_joint_kv_;
            ManiVec      qpik_mani_damping_;
            Vector3d     qpik_base_damping_;
            ManiVec      qpid_mani_vel_damping_;
            ManiVec      qpid_mani_acc_damping_;
            Vector3d     qpid_base_vel_damping_;
            Vector3d     qpid_base_acc_damping_;
            std::map<std::string, Vector6d> link_task_kp_;
            std::map<std::string, Vector6d> link_task_kv_;
            std::map<std::string, Vector6d> link_qpik_tracking_;
            std::map<std::string, Vector6d> link_qpid_tracking_;
    };
} // namespace FR3PCV
