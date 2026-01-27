#include "dyros_robot_menagerie/dual_fr3/controller.h"

namespace DualFR3
{
    void DualFR3Controller::configure(const rclcpp::Node::SharedPtr& node)
    {
        MujocoRosSim::ControllerInterface::configure(node);
        dt_ = 0.001;

        robot_data_ = std::make_shared<DualFR3RobotData>();
        robot_controller_ = std::make_unique<drc::Manipulator::RobotController>(dt_, robot_data_);
        
        key_sub_              = node_->create_subscription<std_msgs::msg::Int32>("dual_fr3_controller/mode_input", 10,std::bind(&DualFR3Controller::keyCallback, this, std::placeholders::_1));
        target_l_ee_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("dual_fr3_controller/target_l_ee_pose", 10,std::bind(&DualFR3Controller::subtargetLEEPoseCallback, this, std::placeholders::_1));
        target_r_ee_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("dual_fr3_controller/target_r_ee_pose", 10,std::bind(&DualFR3Controller::subtargetREEPoseCallback, this, std::placeholders::_1));
        
        current_l_ee_pose_pub_  = node_->create_publisher<geometry_msgs::msg::PoseStamped>("dual_fr3_controller/l_ee_pose", 10);
        current_r_ee_pose_pub_  = node_->create_publisher<geometry_msgs::msg::PoseStamped>("dual_fr3_controller/r_ee_pose", 10);
        
        q_.setZero();
        q_desired_.setZero();
        q_init_.setZero();
        qdot_.setZero();
        qdot_desired_.setZero();
        qdot_init_.setZero();

        x_l_goal_.setIdentity();
        x_r_goal_.setIdentity();
        link_ee_name_l_ = robot_data_->getLEEName();
        link_ee_name_r_ = robot_data_->getREEName();
        link_ee_task_[link_ee_name_l_] = drc::TaskSpaceData::Zero();
        link_ee_task_[link_ee_name_r_] = drc::TaskSpaceData::Zero();
        
        torque_desired_.setZero();


        std::vector<double> joint_kp_vec         = node->declare_parameter<std::vector<double>>("joint_gains.kp",               {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0,
                                                                                                                                 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0});
        std::vector<double> joint_kv_vec         = node->declare_parameter<std::vector<double>>("joint_gains.kv",               {30.0,  30.0,  30.0,  30.0,  10.0,  10.0,  5.0,
                                                                                                                                 30.0,  30.0,  30.0,  30.0,  10.0,  10.0,  5.0});
        std::vector<double> task_kp_l_vec        = node->declare_parameter<std::vector<double>>("task_gains.kp.left",           {100.0, 100.0, 100.0, 100.0, 100.0, 100.0});
        std::vector<double> task_kp_r_vec        = node->declare_parameter<std::vector<double>>("task_gains.kp.right",          {100.0, 100.0, 100.0, 100.0, 100.0, 100.0});
        std::vector<double> task_kv_l_vec        = node->declare_parameter<std::vector<double>>("task_gains.kv.left",           {20.0,  20.0,  20.0,  20.0,  20.0,  20.0});
        std::vector<double> task_kv_r_vec        = node->declare_parameter<std::vector<double>>("task_gains.kv.right",          {20.0,  20.0,  20.0,  20.0,  20.0,  20.0});
        std::vector<double> qpik_tracking_l_vec  = node->declare_parameter<std::vector<double>>("QPIK_gains.tracking.left",     {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        std::vector<double> qpik_tracking_r_vec  = node->declare_parameter<std::vector<double>>("QPIK_gains.tracking.right",    {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        std::vector<double> qpik_damping_vec     = node->declare_parameter<std::vector<double>>("QPIK_gains.damping",           {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                                                                                                                 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        std::vector<double> qpid_tracking_l_vec  = node->declare_parameter<std::vector<double>>("QPID_gains.tracking.left",     {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        std::vector<double> qpid_tracking_r_vec  = node->declare_parameter<std::vector<double>>("QPID_gains.tracking.right",    {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        std::vector<double> qpid_vel_damping_vec = node->declare_parameter<std::vector<double>>("QPID_gains.vel_damping",       {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
                                                                                                                                 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
        std::vector<double> qpid_acc_damping_vec = node->declare_parameter<std::vector<double>>("QPID_gains.acc_damping",       {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                                                                                                                                 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
        
        joint_kp_                            = Eigen::Map<Eigen::VectorXd>(joint_kp_vec.data(),         joint_kp_vec.size());
        joint_kv_                            = Eigen::Map<Eigen::VectorXd>(joint_kv_vec.data(),         joint_kv_vec.size());
        link_task_kp_[link_ee_name_l_]       = Eigen::Map<Eigen::VectorXd>(task_kp_l_vec.data(),        task_kp_l_vec.size());
        link_task_kp_[link_ee_name_r_]       = Eigen::Map<Eigen::VectorXd>(task_kp_r_vec.data(),        task_kp_r_vec.size());
        link_task_kv_[link_ee_name_l_]       = Eigen::Map<Eigen::VectorXd>(task_kv_l_vec.data(),        task_kv_l_vec.size());
        link_task_kv_[link_ee_name_r_]       = Eigen::Map<Eigen::VectorXd>(task_kv_r_vec.data(),        task_kv_r_vec.size());
        link_qpik_tracking_[link_ee_name_l_] = Eigen::Map<Eigen::VectorXd>(qpik_tracking_l_vec.data(),  qpik_tracking_l_vec.size());
        link_qpik_tracking_[link_ee_name_r_] = Eigen::Map<Eigen::VectorXd>(qpik_tracking_r_vec.data(),  qpik_tracking_r_vec.size());
        qpik_damping_                        = Eigen::Map<Eigen::VectorXd>(qpik_damping_vec.data(),     qpik_damping_vec.size());
        link_qpid_tracking_[link_ee_name_l_] = Eigen::Map<Eigen::VectorXd>(qpid_tracking_l_vec.data(),  qpid_tracking_l_vec.size());
        link_qpid_tracking_[link_ee_name_r_] = Eigen::Map<Eigen::VectorXd>(qpid_tracking_r_vec.data(),  qpid_tracking_r_vec.size());
        qpid_vel_damping_                    = Eigen::Map<Eigen::VectorXd>(qpid_vel_damping_vec.data(), qpid_vel_damping_vec.size());
        qpid_acc_damping_                    = Eigen::Map<Eigen::VectorXd>(qpid_acc_damping_vec.data(), qpid_acc_damping_vec.size());

        if (joint_kp_.size()                            != JOINT_DOF) RCLCPP_WARN(node->get_logger(), "joint_gains.kp size mismatch (expected 14)");
        if (joint_kv_.size()                            != JOINT_DOF) RCLCPP_WARN(node->get_logger(), "joint_gains.kv size mismatch (expected 14)");
        if (link_task_kp_[link_ee_name_l_].size()       != TASK_DOF)  RCLCPP_WARN(node->get_logger(), "task_gains.kp.left size mismatch (expected 6)");
        if (link_task_kp_[link_ee_name_r_].size()       != TASK_DOF)  RCLCPP_WARN(node->get_logger(), "task_gains.kp.right size mismatch (expected 6)");
        if (link_task_kv_[link_ee_name_l_].size()       != TASK_DOF)  RCLCPP_WARN(node->get_logger(), "task_gains.kv.left size mismatch (expected 6)");
        if (link_task_kv_[link_ee_name_r_].size()       != TASK_DOF)  RCLCPP_WARN(node->get_logger(), "task_gains.kv.right size mismatch (expected 6)");
        if (link_qpik_tracking_[link_ee_name_l_].size() != TASK_DOF)  RCLCPP_WARN(node->get_logger(), "QPIK_gains.tracking.left size mismatch (expected 6)");
        if (link_qpik_tracking_[link_ee_name_r_].size() != TASK_DOF)  RCLCPP_WARN(node->get_logger(), "QPIK_gains.tracking.right size mismatch (expected 6)");
        if (qpik_damping_.size()                        != JOINT_DOF) RCLCPP_WARN(node->get_logger(), "QPIK_gains.damping size mismatch (expected 14)");
        if (link_qpid_tracking_[link_ee_name_l_].size() != TASK_DOF)  RCLCPP_WARN(node->get_logger(), "QPID_gains.tracking.left size mismatch (expected 6)");
        if (link_qpid_tracking_[link_ee_name_r_].size() != TASK_DOF)  RCLCPP_WARN(node->get_logger(), "QPID_gains.tracking.right size mismatch (expected 6)");
        if (qpid_vel_damping_.size()                    != JOINT_DOF) RCLCPP_WARN(node->get_logger(), "QPID_gains.vel_damping size mismatch (expected 14)");
        if (qpid_acc_damping_.size()                    != JOINT_DOF) RCLCPP_WARN(node->get_logger(), "QPID_gains.acc_damping size mismatch (expected 14)");

        robot_controller_->setJointGain(joint_kp_, joint_kv_);
        robot_controller_->setTaskGain(link_task_kp_, link_task_kv_);
        robot_controller_->setQPIKGain(link_qpik_tracking_, qpik_damping_);
        robot_controller_->setQPIDGain(link_qpid_tracking_, qpid_vel_damping_, qpid_acc_damping_);


        std::ostringstream oss;
        oss << "\n=================================================================\n"
            << "=================================================================\n"
            << "URDF Joint Information: DualFR3\n"
            << robot_data_->getVerbose()
            << "=================================================================\n"
            << "=================================================================";
        const std::string print_info = oss.str();
        RCLCPP_INFO(node->get_logger(), "%s%s%s", cblue, print_info.c_str(), creset);
    }

    void DualFR3Controller::starting()
    {
        current_l_ee_pose_pub_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),  std::bind(&DualFR3Controller::pubLEEPoseCallback, this));
        current_r_ee_pose_pub_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),  std::bind(&DualFR3Controller::pubREEPoseCallback, this));
    }

    void DualFR3Controller::updateState(const MujocoRosSim::VecMap& pos_dict, 
                                        const MujocoRosSim::VecMap& vel_dict,
                                        const MujocoRosSim::VecMap& tau_ext_dict, 
                                        const MujocoRosSim::VecMap& sensors_dict, 
                                        double current_time)
    {
        current_time_ = current_time;

        // get manipulator joint
        for(size_t i=0; i<int(JOINT_DOF/2); i++)
        {
            const std::string& l_name = "left_fr3_joint" + std::to_string(i+1);
            const std::string& r_name = "right_fr3_joint" + std::to_string(i+1);
            q_(i) = pos_dict.at(l_name)(0);
            qdot_(i) = vel_dict.at(l_name)(0);
            q_(i + int(JOINT_DOF/2)) = pos_dict.at(r_name)(0);
            qdot_(i + int(JOINT_DOF/2)) = vel_dict.at(r_name)(0);
        }

        if(!robot_data_->updateState(q_, qdot_))
        {
            RCLCPP_ERROR(node_->get_logger(), "%sFailed to update robot state.%s", cred, creset);
        }

        // get ee
        link_ee_task_[link_ee_name_l_].x    = robot_data_->getPose(link_ee_name_l_);
        link_ee_task_[link_ee_name_r_].x    = robot_data_->getPose(link_ee_name_r_);
        link_ee_task_[link_ee_name_l_].xdot = robot_data_->getVelocity(link_ee_name_l_);
        link_ee_task_[link_ee_name_r_].xdot = robot_data_->getVelocity(link_ee_name_r_);
        
    }

    void DualFR3Controller::updateRGBDImage(const MujocoRosSim::ImageCVMap& images)
    {

    }

    void DualFR3Controller::compute()
    {
        if(is_mode_changed_)
        {
            is_mode_changed_ = false;

            control_start_time_ = current_time_;

            q_init_    = q_;
            qdot_init_ = qdot_;
            q_desired_ = q_init_;
            qdot_desired_.setZero();

            x_l_goal_     = link_ee_task_[link_ee_name_l_].x;
            x_r_goal_     = link_ee_task_[link_ee_name_r_].x;

            link_ee_task_[link_ee_name_l_].setInit();
            link_ee_task_[link_ee_name_l_].setDesired();
            link_ee_task_[link_ee_name_l_].xdot.setZero();
            link_ee_task_[link_ee_name_r_].setInit();
            link_ee_task_[link_ee_name_r_].setDesired();
            link_ee_task_[link_ee_name_r_].xdot.setZero();
        }

        if(mode_ == "CLIK" || mode_ == "OSF" || mode_ == "QPIK" || mode_ == "QPID")
        {
            if(is_l_goal_pose_changed_)
            {
                control_start_time_ = current_time_;
                
                link_ee_task_[link_ee_name_l_].setInit();
                link_ee_task_[link_ee_name_l_].xdot.setZero();
                link_ee_task_[link_ee_name_l_].x_desired = x_l_goal_;

                is_l_goal_pose_changed_ = false;

            }
            if(is_r_goal_pose_changed_)
            {
                control_start_time_ = current_time_;
                
                link_ee_task_[link_ee_name_r_].setInit();
                link_ee_task_[link_ee_name_r_].xdot.setZero();
                link_ee_task_[link_ee_name_r_].x_desired =  x_r_goal_;

                is_r_goal_pose_changed_ = false;
            }
        }
        
        if(mode_ == "HOME")
        {
            JointVec q_target;
            q_target << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4, 
                        0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
            torque_desired_ = robot_controller_->moveJointTorqueCubic(q_target,
                                                                      JointVec::Zero(),
                                                                      q_init_,
                                                                      qdot_init_,
                                                                      current_time_,
                                                                      control_start_time_,
                                                                      4.0,
                                                                      false);
        }
        else if(mode_ == "CLIK")
        {
            qdot_desired_ = robot_controller_->CLIKCubic(link_ee_task_, current_time_, control_start_time_, 4.0);
            q_desired_ += dt_ * qdot_desired_;
            torque_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
        }
        else if(mode_ == "QPIK")
        {
            qdot_desired_ = robot_controller_->QPIKCubic(link_ee_task_, current_time_, control_start_time_, 4.0);
            q_desired_ += dt_ * qdot_desired_;
            torque_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
        }
        else if(mode_ == "OSF")
        {
            JointVec target_q;
            target_q << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4, 
                        0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
            VectorXd null_torque_desired = robot_controller_->moveJointTorqueStep(q_init_, JointVec::Zero());
            torque_desired_ = robot_controller_->OSFCubic(link_ee_task_, current_time_, control_start_time_, 4.0, null_torque_desired);
        }
        else if(mode_ == "QPID")
        {
            torque_desired_ = robot_controller_->QPIDCubic(link_ee_task_, current_time_, control_start_time_, 4.0);
        }
        
        else if(mode_ == "Gravity_compensattion_W_QPID")
        {
            VectorXd qddot_mobile_desired,torque_desired;
            link_ee_task_[link_ee_name_l_].xddot_desired.setZero();
            link_ee_task_[link_ee_name_r_].xddot_desired.setZero();

            torque_desired_ = robot_controller_->QPID(link_ee_task_);
        }
        else
        {
            torque_desired_ = robot_data_->getGravity();
        }
    }

    MujocoRosSim::CtrlInputMap DualFR3Controller::getCtrlInput() const
    {
        MujocoRosSim::CtrlInputMap ctrl_dict;
        for(size_t i=0; i<int(JOINT_DOF/2); i++)
        {
            const std::string l_name = "left_fr3_joint" + std::to_string(i+1);
            const std::string r_name = "right_fr3_joint" + std::to_string(i+1);
            ctrl_dict[l_name] = torque_desired_(i);
            ctrl_dict[r_name] = torque_desired_(int(JOINT_DOF/2)+i);
        }

        return ctrl_dict;
    }

    void DualFR3Controller::setMode(const std::string& mode)
    {
        is_mode_changed_ = true;
        mode_ = mode;
        RCLCPP_INFO(node_->get_logger(), "\033[34m Mode changed: %s\033[0m", mode.c_str());
    }

    void DualFR3Controller::keyCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Key input received: %d", msg->data);
        if(msg->data == 1)      setMode("HOME");
        else if(msg->data == 2) setMode("CLIK");
        else if(msg->data == 3) setMode("QPIK");
        else if(msg->data == 4) setMode("OSF");
        else if(msg->data == 5) setMode("QPID");
        else if(msg->data == 6) setMode("Gravity_compensattion_W_QPID");
        else                    setMode("NONE");
    }
    
    void DualFR3Controller::subtargetLEEPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Target ee pose received: position=(%.3f, %.3f, %.3f), "
                    "orientation=(%.3f, %.3f, %.3f, %.3f)",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                    msg->pose.orientation.x, msg->pose.orientation.y,
                    msg->pose.orientation.z, msg->pose.orientation.w);

        // Convert to 4x4 homogeneous transform
        Eigen::Quaterniond quat(msg->pose.orientation.w,
                                msg->pose.orientation.x,
                                msg->pose.orientation.y,
                                msg->pose.orientation.z);

        x_l_goal_.linear() = quat.toRotationMatrix();
        x_l_goal_.translation() << msg->pose.position.x,
                                   msg->pose.position.y,
                                   msg->pose.position.z;
        is_l_goal_pose_changed_ = true;
    }

    void DualFR3Controller::subtargetREEPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Target right ee pose received: position=(%.3f, %.3f, %.3f), "
                    "orientation=(%.3f, %.3f, %.3f, %.3f)",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                    msg->pose.orientation.x, msg->pose.orientation.y,
                    msg->pose.orientation.z, msg->pose.orientation.w);

        // Convert to 4x4 homogeneous transform
        Eigen::Quaterniond quat(msg->pose.orientation.w,
                                msg->pose.orientation.x,
                                msg->pose.orientation.y,
                                msg->pose.orientation.z);

         x_r_goal_.linear() = quat.toRotationMatrix();
         x_r_goal_.translation() << msg->pose.position.x,
                                    msg->pose.position.y,
                                    msg->pose.position.z;
        is_r_goal_pose_changed_ = true;
    }

    void DualFR3Controller::pubLEEPoseCallback()
    {
        auto ee_pose_msg = geometry_msgs::msg::PoseStamped();
        ee_pose_msg.header.frame_id = "base_link";
        ee_pose_msg.header.stamp = node_->now();

        ee_pose_msg.pose.position.x = link_ee_task_[link_ee_name_l_].x.translation()(0);
        ee_pose_msg.pose.position.y = link_ee_task_[link_ee_name_l_].x.translation()(1);
        ee_pose_msg.pose.position.z = link_ee_task_[link_ee_name_l_].x.translation()(2);

        Eigen::Quaterniond q(link_ee_task_[link_ee_name_l_].x.rotation());
        ee_pose_msg.pose.orientation.x = q.x();
        ee_pose_msg.pose.orientation.y = q.y();
        ee_pose_msg.pose.orientation.z = q.z();
        ee_pose_msg.pose.orientation.w = q.w();
        
        current_l_ee_pose_pub_->publish(ee_pose_msg);
    }

    void DualFR3Controller::pubREEPoseCallback()
    {
        auto ee_pose_msg = geometry_msgs::msg::PoseStamped();
        ee_pose_msg.header.frame_id = "base_link";
        ee_pose_msg.header.stamp = node_->now();

        ee_pose_msg.pose.position.x =  link_ee_task_[link_ee_name_r_].x.translation()(0);
        ee_pose_msg.pose.position.y =  link_ee_task_[link_ee_name_r_].x.translation()(1);
        ee_pose_msg.pose.position.z =  link_ee_task_[link_ee_name_r_].x.translation()(2);

        Eigen::Quaterniond q( link_ee_task_[link_ee_name_r_].x.rotation());
        ee_pose_msg.pose.orientation.x = q.x();
        ee_pose_msg.pose.orientation.y = q.y();
        ee_pose_msg.pose.orientation.z = q.z();
        ee_pose_msg.pose.orientation.w = q.w();
        
        current_r_ee_pose_pub_->publish(ee_pose_msg);
    }

    /* register with the global registry */
    PLUGINLIB_EXPORT_CLASS(DualFR3::DualFR3Controller, MujocoRosSim::ControllerInterface)
} // namespace DualFR3