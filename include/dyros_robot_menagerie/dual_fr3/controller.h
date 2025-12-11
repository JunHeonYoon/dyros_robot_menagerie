#pragma once
#include <pluginlib/class_list_macros.hpp>
#include "mujoco_ros_sim/controller_interface.hpp"

#include "dyros_robot_menagerie/dual_fr3/robot_data.h"

#include "dyros_robot_controller/manipulator/robot_controller.h"

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

namespace DualFR3
{
/*
MuJoCo Model Information: dual_fr3
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  0 | fr3_l_joint1         | Hinge  |  1 |  1 |     0 |    0
  1 | fr3_l_joint2         | Hinge  |  1 |  1 |     1 |    1
  2 | fr3_l_joint3         | Hinge  |  1 |  1 |     2 |    2
  3 | fr3_l_joint4         | Hinge  |  1 |  1 |     3 |    3
  4 | fr3_l_joint5         | Hinge  |  1 |  1 |     4 |    4
  5 | fr3_l_joint6         | Hinge  |  1 |  1 |     5 |    5
  6 | fr3_l_joint7         | Hinge  |  1 |  1 |     6 |    6
  7 | fr3_l_finger_joint1  | Slide  |  1 |  1 |     7 |    7
  8 | fr3_l_finger_joint2  | Slide  |  1 |  1 |     8 |    8
  9 | fr3_r_joint1         | Hinge  |  1 |  1 |     9 |    9
 10 | fr3_r_joint2         | Hinge  |  1 |  1 |    10 |   10
 11 | fr3_r_joint3         | Hinge  |  1 |  1 |    11 |   11
 12 | fr3_r_joint4         | Hinge  |  1 |  1 |    12 |   12
 13 | fr3_r_joint5         | Hinge  |  1 |  1 |    13 |   13
 14 | fr3_r_joint6         | Hinge  |  1 |  1 |    14 |   14
 15 | fr3_r_joint7         | Hinge  |  1 |  1 |    15 |   15
 16 | fr3_r_finger_joint1  | Slide  |  1 |  1 |    16 |   16
 17 | fr3_r_finger_joint2  | Slide  |  1 |  1 |    17 |   17

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | fr3_l_joint1         | Joint   | fr3_l_joint1
  1 | fr3_l_joint2         | Joint   | fr3_l_joint2
  2 | fr3_l_joint3         | Joint   | fr3_l_joint3
  3 | fr3_l_joint4         | Joint   | fr3_l_joint4
  4 | fr3_l_joint5         | Joint   | fr3_l_joint5
  5 | fr3_l_joint6         | Joint   | fr3_l_joint6
  6 | fr3_l_joint7         | Joint   | fr3_l_joint7
  7 | fr3_r_joint1         | Joint   | fr3_r_joint1
  8 | fr3_r_joint2         | Joint   | fr3_r_joint2
  9 | fr3_r_joint3         | Joint   | fr3_r_joint3
 10 | fr3_r_joint4         | Joint   | fr3_r_joint4
 11 | fr3_r_joint5         | Joint   | fr3_r_joint5
 12 | fr3_r_joint6         | Joint   | fr3_r_joint6
 13 | fr3_r_joint7         | Joint   | fr3_r_joint7
 14 | fr3_l_hand           | Tendon  | fr3_l_joint1
 15 | fr3_r_hand           | Tendon  | fr3_l_joint2

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------

 id | name                        | mode     | resolution
----+-----------------------------+----------+------------
  0 | l_realsense_camera          | -        | 640x480
  1 | r_realsense_camera          | -        | 640x480
*/
    class DualFR3Controller final : public MujocoRosSim::ControllerInterface
    {
        public:
            DualFR3Controller() = default;
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
            void pubREEPoseCallback();
            void pubLEEPoseCallback();

            std::shared_ptr<DualFR3::DualFR3RobotData> robot_data_;
            std::unique_ptr<drc::Manipulator::RobotController> robot_controller_;

            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            key_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_l_ee_pose_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_r_ee_pose_sub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    current_l_ee_pose_pub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    current_r_ee_pose_pub_;

            rclcpp::TimerBase::SharedPtr current_l_ee_pose_pub_timer_;
            rclcpp::TimerBase::SharedPtr current_r_ee_pose_pub_timer_;

            bool is_mode_changed_{true};
            bool is_l_goal_pose_changed_{false};
            bool is_r_goal_pose_changed_{false};

            std::string mode_{"HOME"};
            
            double control_start_time_;
            double current_time_;
            
            //// joint space state
            JointVec q_;
            JointVec q_desired_;
            JointVec q_init_;
            JointVec qdot_;
            JointVec qdot_desired_;
            JointVec qdot_init_;

            //// operation space state
            // left
            std::string link_ee_name_l_;
            Affine3d x_l_goal_;
 
            // right
            std::string link_ee_name_r_;
            Affine3d x_r_goal_;

            std::map<std::string, drc::TaskSpaceData> link_ee_task_;

            //// control input
            JointVec torque_desired_;

            //// gains
            JointVec joint_kp_;
            JointVec joint_kv_;
            JointVec qpik_damping_; 
            JointVec qpid_vel_damping_;
            JointVec qpid_acc_damping_;
            std::map<std::string, Vector6d> link_task_kp_;
            std::map<std::string, Vector6d> link_task_kv_;
            std::map<std::string, Vector6d> link_qpik_tracking_;
            std::map<std::string, Vector6d> link_qpid_tracking_;
    };
} // namespace DualFR3