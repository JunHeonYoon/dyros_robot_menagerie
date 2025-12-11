#pragma once
#include <pluginlib/class_list_macros.hpp>
#include "mujoco_ros_sim/controller_interface.hpp"

#include "dyros_robot_menagerie/fr3/robot_data.h"

#include "dyros_robot_controller/manipulator/robot_controller.h"

#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include "image_io.h"
#include "math_type_define.h"

#include <mutex> 
#include <algorithm>

namespace FR3
{
/*
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
*/
    
    class FR3Controller final : public MujocoRosSim::ControllerInterface
    {
        public:
            FR3Controller() = default;
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
            void subtargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);
            void pubEEPoseCallback();
            void pubHandEyeCallback();

            std::shared_ptr<FR3RobotData> robot_data_;
            std::unique_ptr<drc::Manipulator::RobotController> robot_controller_;

            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            key_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    ee_pose_pub_;
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr            hand_eye_rgb_pub_;
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr            hand_eye_depth_pub_;
            
            rclcpp::TimerBase::SharedPtr ee_pose_pub_timer_;
            rclcpp::TimerBase::SharedPtr hand_eye_cam_pub_timer_;

            cv::Mat hand_eye_rgb_img_;
            cv::Mat hand_eye_depth_img_;
            std::mutex hand_eye_cam_mtx_;
    
            bool is_mode_changed_{true};
            bool is_goal_pose_changed_{false};

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
            std::string link_ee_name_;            
            Affine3d x_goal_;
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
} // namespace FR3Controller