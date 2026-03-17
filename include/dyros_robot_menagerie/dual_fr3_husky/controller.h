#pragma once
#include <pluginlib/class_list_macros.hpp>
#include "mujoco_ros_sim/controller_interface.hpp"

#include "dyros_robot_menagerie/dual_fr3_husky/robot_data.h"

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

namespace DualFR3Husky
{
/*
MuJoCo Model Information: dual_fr3_husky
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  0 | -                    | Free   |  7 |  6 |     0 |    0
  1 | front_left_wheel     | Hinge  |  1 |  1 |     7 |    6
  2 | front_right_wheel    | Hinge  |  1 |  1 |     8 |    7
  3 | rear_left_wheel      | Hinge  |  1 |  1 |     9 |    8
  4 | rear_right_wheel     | Hinge  |  1 |  1 |    10 |    9
  5 | fr3_l_joint1         | Hinge  |  1 |  1 |    11 |   10
  6 | fr3_l_joint2         | Hinge  |  1 |  1 |    12 |   11
  7 | fr3_l_joint3         | Hinge  |  1 |  1 |    13 |   12
  8 | fr3_l_joint4         | Hinge  |  1 |  1 |    14 |   13
  9 | fr3_l_joint5         | Hinge  |  1 |  1 |    15 |   14
 10 | fr3_l_joint6         | Hinge  |  1 |  1 |    16 |   15
 11 | fr3_l_joint7         | Hinge  |  1 |  1 |    17 |   16
 12 | fr3_r_joint1         | Hinge  |  1 |  1 |    18 |   17
 13 | fr3_r_joint2         | Hinge  |  1 |  1 |    19 |   18
 14 | fr3_r_joint3         | Hinge  |  1 |  1 |    20 |   19
 15 | fr3_r_joint4         | Hinge  |  1 |  1 |    21 |   20
 16 | fr3_r_joint5         | Hinge  |  1 |  1 |    22 |   21
 17 | fr3_r_joint6         | Hinge  |  1 |  1 |    23 |   22
 18 | fr3_r_joint7         | Hinge  |  1 |  1 |    24 |   23

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | left_wheel           | Joint   | front_left_wheel
  1 | right_wheel          | Joint   | front_right_wheel
  2 | fr3_l_joint1         | Joint   | fr3_l_joint1
  3 | fr3_l_joint2         | Joint   | fr3_l_joint2
  4 | fr3_l_joint3         | Joint   | fr3_l_joint3
  5 | fr3_l_joint4         | Joint   | fr3_l_joint4
  6 | fr3_l_joint5         | Joint   | fr3_l_joint5
  7 | fr3_l_joint6         | Joint   | fr3_l_joint6
  8 | fr3_l_joint7         | Joint   | fr3_l_joint7
  9 | fr3_r_joint1         | Joint   | fr3_r_joint1
 10 | fr3_r_joint2         | Joint   | fr3_r_joint2
 11 | fr3_r_joint3         | Joint   | fr3_r_joint3
 12 | fr3_r_joint4         | Joint   | fr3_r_joint4
 13 | fr3_r_joint5         | Joint   | fr3_r_joint5
 14 | fr3_r_joint6         | Joint   | fr3_r_joint6
 15 | fr3_r_joint7         | Joint   | fr3_r_joint7

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
  0 | position_sensor             | FramePos         |   3 |   0 | Site:husky_site
  1 | orientation_sensor          | FrameQuat        |   4 |   3 | Site:husky_site
  2 | linear_velocity_sensor      | FrameLinVel      |   3 |   7 | Site:husky_site
  3 | angular_velocity_sensor     | FrameAngVel      |   3 |  10 | Site:husky_site

 id | name                        | mode     | resolution
----+-----------------------------+----------+------------
  0 | l_realsense_camera          | -        | 640x480
  1 | r_realsense_camera          | -        | 640x480
*/
    class DualFR3HuskyController final : public MujocoRosSim::ControllerInterface
    {
        public:
            DualFR3HuskyController() = default;
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
            void subtargetLEEPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);
            void subtargetREEPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);
            void subtargetBaseVelCallback(const geometry_msgs::msg::Twist::SharedPtr);
            void subJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr);
            void pubREEPoseCallback();
            void pubLEEPoseCallback();
            void pubBasePoseCallback();
            void pubBaseVelCallback();

            std::shared_ptr<DualFR3Husky::DualFR3HuskyRobotData> robot_data_;
            std::unique_ptr<drc::MobileManipulator::RobotController> robot_controller_;

            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            key_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_l_ee_pose_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_r_ee_pose_sub_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       target_base_vel_sub_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr    joint_sub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    current_l_ee_pose_pub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    current_r_ee_pose_pub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    current_base_pose_pub_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          current_base_vel_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr       joint_pub_;

            rclcpp::TimerBase::SharedPtr current_l_ee_pose_pub_timer_;
            rclcpp::TimerBase::SharedPtr current_r_ee_pose_pub_timer_;
            rclcpp::TimerBase::SharedPtr current_base_pose_pub_timer_;
            rclcpp::TimerBase::SharedPtr current_base_vel_pub_timer_;

            bool is_mode_changed_{true};
            bool is_l_goal_pose_changed_{false};
            bool is_r_goal_pose_changed_{false};

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
            // left
            std::string link_ee_name_l_;
            Affine3d x_l_goal_;
 
            // right
            std::string link_ee_name_r_;
            Affine3d x_r_goal_;

            std::map<std::string, drc::TaskSpaceData> link_ee_task_;

            //// control input
            ManiVec torque_mani_desired_;
            MobiVec qdot_mobile_desired_;

            //// gains
            ManiVec      mani_joint_kp_;
            ManiVec      mani_joint_kv_;
            Vector6d     ik_kp_;
            Vector6d     id_kp_;
            Vector6d     id_kv_;
            Vector6d     qpik_tracking_;
            ManiVec      qpik_mani_damping_;
            Vector3d     qpik_base_damping_;
            ManiVec      qpik_mani_acc_damping_;
            Vector3d     qpik_base_acc_damping_;
            Vector6d     qpid_tracking_;
            ManiVec      qpid_mani_vel_damping_;
            ManiVec      qpid_mani_acc_damping_;
            Vector3d     qpid_base_vel_damping_;
            Vector3d     qpid_base_acc_damping_;
    };
} // namespace DualFR3Husky
