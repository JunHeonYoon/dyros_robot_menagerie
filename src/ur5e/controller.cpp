#include "dyros_robot_menagerie/ur5e/controller.h"

namespace UR5e
{
    void UR5eController::configure(const rclcpp::Node::SharedPtr& node)
    {
        MujocoRosSim::ControllerInterface::configure(node);
        dt_ = 0.01;

        robot_data_ = std::make_shared<UR5eRobotData>();
        robot_controller_ = std::make_unique<drc::Manipulator::RobotController>(dt_, robot_data_);

        rclcpp::QoS qos(rclcpp::KeepLast(1)); 
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort); 
        qos.durability(rclcpp::DurabilityPolicy::Volatile);
        
        key_sub_ = node_->create_subscription<std_msgs::msg::Int32>("ur5e_controller/mode_input", 10,std::bind(&UR5eController::keyCallback, this, std::placeholders::_1));
        target_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("ur5e_controller/target_pose", 10,std::bind(&UR5eController::subtargetPoseCallback, this, std::placeholders::_1));
        
        ee_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("ur5e_controller/ee_pose", 10);
        
        q_.setZero();
        q_init_.setZero();
        qdot_.setZero();
        qdot_desired_.setZero();
        qdot_init_.setZero();
        
        x_.setIdentity();
        x_init_.setIdentity();
        x_desired_.setIdentity();
        xdot_.setZero();
        xdot_init_.setZero();
        xdot_desired_.setZero();
        
        x_goal_.setIdentity();
        
        q_desired_.setZero();
        
        std::ostringstream oss;
        oss << "\n=================================================================\n"
            << "=================================================================\n"
            << "URDF Joint Information: UR5e\n"
            << robot_data_->getVerbose()
            << "=================================================================\n"
            << "=================================================================";
        const std::string print_info = oss.str();
        RCLCPP_INFO(node->get_logger(), "%s%s%s", cblue, print_info.c_str(), creset);
    }
    
    void UR5eController::starting()
    {
        ee_pose_pub_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), std::bind(&UR5eController::pubEEPoseCallback, this));
    }

    void UR5eController::updateState(const MujocoRosSim::VecMap& pos_dict, 
                                     const MujocoRosSim::VecMap& vel_dict,
                                     const MujocoRosSim::VecMap& tau_ext_dict, 
                                     const MujocoRosSim::VecMap& sensors_dict, 
                                     double current_time)
    {
        current_time_ = current_time;

        // get manipulator joint
        q_(0) = pos_dict.at("shoulder_pan_joint")(0);  qdot_(0) = vel_dict.at("shoulder_pan_joint")(0);
        q_(1) = pos_dict.at("shoulder_lift_joint")(0); qdot_(1) = vel_dict.at("shoulder_lift_joint")(0);
        q_(2) = pos_dict.at("elbow_joint")(0);         qdot_(2) = vel_dict.at("elbow_joint")(0);
        q_(3) = pos_dict.at("wrist_1_joint")(0);       qdot_(3) = vel_dict.at("wrist_1_joint")(0);
        q_(4) = pos_dict.at("wrist_2_joint")(0);       qdot_(4) = vel_dict.at("wrist_2_joint")(0);
        q_(5) = pos_dict.at("wrist_3_joint")(0);       qdot_(5) = vel_dict.at("wrist_3_joint")(0);

        if(!robot_data_->updateState(q_, qdot_)) RCLCPP_ERROR(node_->get_logger(), "%sFailed to update robot state.%s", cred, creset);

        // get ee
        x_ = robot_data_->getPose();
        xdot_ = robot_data_->getVelocity();
    }

    void UR5eController::updateRGBDImage(const MujocoRosSim::ImageCVMap& images)
    {
    }

    void UR5eController::compute()
    {
        if(is_mode_changed_)
        {
            is_mode_changed_ = false;
            control_start_time_ = current_time_;

            q_init_ = q_;
            qdot_init_ = qdot_;
            q_desired_ = q_init_;
            qdot_desired_.setZero();

            x_init_ = x_;
            xdot_init_ = xdot_;
            x_desired_ = x_init_;
            xdot_desired_.setZero();

            x_goal_ = x_init_;
        }

        if(mode_ == "HOME")
        {
            JointVec target_q;
            target_q << -M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, 0;
            q_desired_ = robot_controller_->moveJointPositionCubic(target_q,
                                                                   JointVec::Zero(),
                                                                   q_init_,
                                                                   qdot_init_,
                                                                   current_time_,
                                                                   control_start_time_,
                                                                   4.0);
        }
        else if(mode_ == "CLIK")
        {
            if(is_goal_pose_changed_)
            {
                control_start_time_ = current_time_;
                x_init_ = x_;
                xdot_init_ = xdot_;
                is_goal_pose_changed_ = false;
            }
            qdot_desired_ = robot_controller_->CLIKCubic(x_goal_,
                                                         JointVec::Zero(),
                                                         x_init_,
                                                         xdot_init_,
                                                         current_time_,
                                                         control_start_time_,
                                                         4.0,
                                                         robot_data_->getEEName());
            q_desired_ += dt_ * qdot_desired_;
        }
        else if(mode_ == "QPIK")
        {
            if(is_goal_pose_changed_)
            {
                control_start_time_ = current_time_;
                x_init_ = x_;
                xdot_init_ = xdot_;
                is_goal_pose_changed_ = false;
            }
            qdot_desired_ = robot_controller_->QPIKCubic(x_goal_,
                                                         JointVec::Zero(),
                                                         x_init_,
                                                         xdot_init_,
                                                         current_time_,
                                                         control_start_time_,
                                                         4.0,
                                                         robot_data_->getEEName());
            q_desired_ += dt_ * qdot_desired_;
        }
        else
        {
            q_desired_ = q_init_;
        }
    }

    MujocoRosSim::CtrlInputMap UR5eController::getCtrlInput() const
    {
        MujocoRosSim::CtrlInputMap ctrl_dict;
        ctrl_dict["shoulder_pan"]  = q_desired_(0);
        ctrl_dict["shoulder_lift"] = q_desired_(1);
        ctrl_dict["elbow"]         = q_desired_(2);
        ctrl_dict["wrist_1"]       = q_desired_(3);
        ctrl_dict["wrist_2"]       = q_desired_(4);
        ctrl_dict["wrist_3"]       = q_desired_(5);
        return ctrl_dict;
    }

    void UR5eController::setMode(const std::string& mode)
    {
        is_mode_changed_ = true;
        mode_ = mode;
        RCLCPP_INFO(node_->get_logger(), "%sMode changed: %s%s", cblue, mode.c_str(), creset);
    }

    void UR5eController::keyCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Key input received: %d", msg->data);
        if(msg->data == 1)      setMode("HOME");
        else if(msg->data == 2) setMode("CLIK");
        else if(msg->data == 3) setMode("QPIK");
        else                    setMode("NONE");
    }

    void UR5eController::subtargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Target pose received: position=(%.3f, %.3f, %.3f), "
                    "orientation=(%.3f, %.3f, %.3f, %.3f)",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                    msg->pose.orientation.x, msg->pose.orientation.y,
                    msg->pose.orientation.z, msg->pose.orientation.w);

        // Convert to 4x4 homogeneous transform
        Eigen::Quaterniond quat(msg->pose.orientation.w,
                                msg->pose.orientation.x,
                                msg->pose.orientation.y,
                                msg->pose.orientation.z);

        x_goal_.linear() = quat.toRotationMatrix();
        x_goal_.translation() << msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z;
        is_goal_pose_changed_ = true;
    }

    void UR5eController::pubEEPoseCallback()
    {
        auto ee_pose_msg = geometry_msgs::msg::PoseStamped();
        ee_pose_msg.header.frame_id = "base_link";
        ee_pose_msg.header.stamp = node_->now();

        ee_pose_msg.pose.position.x = x_.translation()(0);
        ee_pose_msg.pose.position.y = x_.translation()(1);
        ee_pose_msg.pose.position.z = x_.translation()(2);

        Eigen::Quaterniond q(x_.rotation());
        ee_pose_msg.pose.orientation.x = q.x();
        ee_pose_msg.pose.orientation.y = q.y();
        ee_pose_msg.pose.orientation.z = q.z();
        ee_pose_msg.pose.orientation.w = q.w();
        
        ee_pose_pub_->publish(ee_pose_msg);
    }

    /* register with the global registry */
    PLUGINLIB_EXPORT_CLASS(UR5e::UR5eController, MujocoRosSim::ControllerInterface)
} // namespace UR5e