#pragma once
#include <pluginlib/class_list_macros.hpp>
#include "mujoco_ros_sim/controller_interface.hpp"

#include "dyros_robot_menagerie/ur5e/robot_data.h"

#include "dyros_robot_controller/manipulator/robot_controller.h"

#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "image_io.h"
#include "math_type_define.h"

#include <mutex> 
#include <algorithm>

namespace UR5e
{
/*
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
*/
    
    class UR5eController final : public MujocoRosSim::ControllerInterface
    {
        public:
            UR5eController() = default;
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

            std::shared_ptr<UR5eRobotData> robot_data_;
            std::unique_ptr<drc::Manipulator::RobotController> robot_controller_;

            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            key_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    ee_pose_pub_;
            
            rclcpp::TimerBase::SharedPtr ee_pose_pub_timer_;
    
            bool is_mode_changed_{true};
            bool is_goal_pose_changed_{false};

            std::string mode_{"HOME"};
            
            double control_start_time_;
            double current_time_;

            //// joint space state
            JointVec q_;
            JointVec q_init_;
            JointVec qdot_;
            JointVec qdot_desired_;
            JointVec qdot_init_;
            
            //// operation space state
            Affine3d x_;
            Affine3d x_desired_;
            Affine3d x_init_;
            TaskVec xdot_;
            TaskVec xdot_desired_;
            TaskVec xdot_init_;
            
            Affine3d x_goal_;

            //// control input
            JointVec q_desired_;
    };
} // namespace UR5e