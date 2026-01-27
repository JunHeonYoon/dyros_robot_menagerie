#include "dyros_robot_menagerie/ur5e/controller.h"

namespace UR5e
{
    void UR5eController::configure(const rclcpp::Node::SharedPtr& node)
    {
        MujocoRosSim::ControllerInterface::configure(node);
        const double dt = 0.01;

        robot_data_ = std::make_shared<UR5eRobotData>(dt);
        dt_ = robot_data_->getDt();
        robot_controller_ = std::make_unique<drc::Manipulator::RobotController>(robot_data_);

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
        
        x_goal_.setIdentity();
        link_ee_name_ = robot_data_->getEEName();
        link_ee_task_[link_ee_name_] = drc::TaskSpaceData::Zero();
        
        q_desired_.setZero();

        std::vector<double> task_kp_vec       = node->declare_parameter<std::vector<double>>("task_gains.kp",       {100.0, 100.0, 100.0, 100.0, 100.0, 100.0});
        std::vector<double> task_kv_vec       = node->declare_parameter<std::vector<double>>("task_gains.kv",       {20.0,  20.0,  20.0,  20.0,  20.0,  20.0});
        std::vector<double> qpik_tracking_vec = node->declare_parameter<std::vector<double>>("QPIK_gains.tracking", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        std::vector<double> qpik_damping_vec  = node->declare_parameter<std::vector<double>>("QPIK_gains.damping",  {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

        link_task_kp_[link_ee_name_]       = Eigen::Map<Eigen::VectorXd>(task_kp_vec.data(),       task_kp_vec.size());
        link_task_kv_[link_ee_name_]       = Eigen::Map<Eigen::VectorXd>(task_kv_vec.data(),       task_kv_vec.size());
        link_qpik_tracking_[link_ee_name_] = Eigen::Map<Eigen::VectorXd>(qpik_tracking_vec.data(), qpik_tracking_vec.size());
        qpik_damping_                      = Eigen::Map<Eigen::VectorXd>(qpik_damping_vec.data(),  qpik_damping_vec.size());

        if (link_task_kp_[link_ee_name_].size()       != TASK_DOF)  RCLCPP_ERROR(node->get_logger(), "task_gains.kp size mismatch (expected 6)");
        if (link_task_kv_[link_ee_name_].size()       != TASK_DOF)  RCLCPP_ERROR(node->get_logger(), "task_gains.kv size mismatch (expected 6)");
        if (link_qpik_tracking_[link_ee_name_].size() != TASK_DOF)  RCLCPP_ERROR(node->get_logger(), "QPIK_gains.tracking size mismatch (expected 6)");
        if (qpik_damping_.size()                      != JOINT_DOF) RCLCPP_ERROR(node->get_logger(), "QPIK_gains.damping size mismatch (expected 6)");

        robot_controller_->setTaskGain(link_task_kp_, link_task_kv_);
        robot_controller_->setQPIKGain(link_qpik_tracking_, qpik_damping_);
        
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
        link_ee_task_[link_ee_name_].x    = robot_data_->getPose();
        link_ee_task_[link_ee_name_].xdot = robot_data_->getVelocity();
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

            x_goal_ = link_ee_task_[link_ee_name_].x;

            link_ee_task_[link_ee_name_].setInit();
            link_ee_task_[link_ee_name_].setDesired();
            link_ee_task_[link_ee_name_].xdot.setZero();
        }

        if(mode_ == "CLIK" || mode_ == "QPIK")
        {
            if(is_goal_pose_changed_)
            {
                control_start_time_ = current_time_;
                
                link_ee_task_[link_ee_name_].setInit();
                link_ee_task_[link_ee_name_].xdot.setZero();
                link_ee_task_[link_ee_name_].x_desired = x_goal_;

                is_goal_pose_changed_ = false;
            }
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
            qdot_desired_ = robot_controller_->CLIKCubic(link_ee_task_, current_time_, control_start_time_, 4.0);
            q_desired_ += dt_ * qdot_desired_;
        }
        else if(mode_ == "QPIK")
        {
            qdot_desired_ = robot_controller_->QPIKCubic(link_ee_task_, current_time_, control_start_time_, 4.0);
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

        ee_pose_msg.pose.position.x = link_ee_task_[link_ee_name_].x.translation()(0);
        ee_pose_msg.pose.position.y = link_ee_task_[link_ee_name_].x.translation()(1);
        ee_pose_msg.pose.position.z = link_ee_task_[link_ee_name_].x.translation()(2);

        Eigen::Quaterniond q(link_ee_task_[link_ee_name_].x.rotation());
        ee_pose_msg.pose.orientation.x = q.x();
        ee_pose_msg.pose.orientation.y = q.y();
        ee_pose_msg.pose.orientation.z = q.z();
        ee_pose_msg.pose.orientation.w = q.w();
        
        ee_pose_pub_->publish(ee_pose_msg);
    }

    /* register with the global registry */
    PLUGINLIB_EXPORT_CLASS(UR5e::UR5eController, MujocoRosSim::ControllerInterface)
} // namespace UR5e
