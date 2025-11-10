#pragma once
#include <pluginlib/class_list_macros.hpp>
#include "mujoco_ros_sim/controller_interface.hpp"

#include "dyros_robot_menagerie/pcv/robot_data.h"

#include "dyros_robot_controller/mobile/robot_controller.h"
#include "math_type_define.h"

#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace PCV
{
/*
MuJoCo Model Information: dyros_pcv
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  1 | front_left_steer     | _Hinge |  1 |  1 |     7 |    6
  2 | front_left_rotate    | _Hinge |  1 |  1 |     8 |    7
  3 | rear_left_steer      | _Hinge |  1 |  1 |     9 |    8
  4 | rear_left_rotate     | _Hinge |  1 |  1 |    10 |    9
  5 | rear_right_steer     | _Hinge |  1 |  1 |    11 |   10
  6 | rear_right_rotate    | _Hinge |  1 |  1 |    12 |   11
  7 | front_right_steer    | _Hinge |  1 |  1 |    13 |   12
  8 | front_right_rotate   | _Hinge |  1 |  1 |    14 |   13

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | front_left_rotate    | _Joint  | front_left_rotate
  1 | front_right_rotate   | _Joint  | front_right_rotate
  2 | rear_left_rotate     | _Joint  | rear_left_rotate
  3 | rear_right_rotate    | _Joint  | rear_right_rotate
  4 | front_left_steer     | _Joint  | front_left_steer
  5 | front_right_steer    | _Joint  | front_right_steer
  6 | rear_left_steer      | _Joint  | rear_left_steer
  7 | rear_right_steer     | _Joint  | rear_right_steer

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
  0 | position_sensor             | Framepos         |   3 |   0 | Site:dyros_pcv_site
  1 | orientation_sensor          | Framequat        |   4 |   3 | Site:dyros_pcv_site
  2 | linear_velocity_sensor      | Framelinvel      |   3 |   7 | Site:dyros_pcv_site
  3 | angular_velocity_sensor     | Frameangvel      |   3 |  10 | Site:dyros_pcv_site

 id | name                        | mode     | resolution
----+-----------------------------+----------+------------
*/

    class PCVController : public MujocoRosSim::ControllerInterface
    {
        public:
            PCVController() = default;
            
            // ====================================================================================
            // ================================== Core Functions ================================== 
            // ====================================================================================
            void configure(const rclcpp::Node::SharedPtr& node) override;
            void starting() override;
            void updateState(const MujocoRosSim::VecMap&, const MujocoRosSim::VecMap&, const MujocoRosSim::VecMap&, const MujocoRosSim::VecMap&, double) override;
            void compute() override;
            MujocoRosSim::CtrlInputMap getCtrlInput() const override;

        private:
            // ====================================================================================
            // ===================== Helper / CB / Background Thread Functions ==================== 
            // ====================================================================================
            void setMode(const std::string& mode);
            void keyCallback(const std_msgs::msg::Int32::SharedPtr);
            void subtargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);
            void subtargetVelCallback(const geometry_msgs::msg::Twist::SharedPtr);
            void subJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr);
            void pubBasePoseCallback();
            void pubBaseVelCallback();

            std::shared_ptr<PCVRobotData> robot_data_;
            std::unique_ptr<drc::Mobile::RobotController> robot_controller_;

            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            key_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       target_vel_sub_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr    joint_sub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    base_pose_pub_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          base_vel_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr       joint_pub_;

            rclcpp::TimerBase::SharedPtr base_pose_pub_timer_;
            rclcpp::TimerBase::SharedPtr base_vel_pub_timer_;

            bool is_mode_changed_{true};
            bool is_goal_pose_changed_{false};

            std::string mode_{"Stop"};
            
            double control_start_time_;
            double current_time_;

            //// Mobile Base pose wrt world frame
            TaskVec base_pose_;
            TaskVec base_pose_desired_;
            TaskVec base_pose_init_;
            
            //// Mobile Base velocity wrt base frame
            TaskVec base_vel_;
            TaskVec base_vel_tmp;
            TaskVec base_vel_desired_;
            TaskVec base_vel_init_;

            WheelVec wheel_pos_;
            WheelVec wheel_vel_;

            //// control input
            WheelVec wheel_vel_desired_; // (front_left(steer, rotate), front_right, rear_left, rear_right)
    };
} // namespace PCV