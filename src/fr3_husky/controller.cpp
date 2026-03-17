#include "dyros_robot_menagerie/fr3_husky/controller.h"

namespace FR3Husky
{
    void FR3HuskyController::configure(const rclcpp::Node::SharedPtr& node)
    {
        MujocoRosSim::ControllerInterface::configure(node);
        const double dt = 0.001;

        robot_data_ = std::make_shared<FR3HuskyRobotData>(dt);
        dt_ = robot_data_->getDt();
        robot_controller_ = std::make_unique<drc::MobileManipulator::RobotController>(robot_data_);
        
        key_sub_             = node_->create_subscription<std_msgs::msg::Int32>("fr3_husky_controller/mode_input", 10,std::bind(&FR3HuskyController::keyCallback, this, std::placeholders::_1));
        target_ee_pose_sub_  = node_->create_subscription<geometry_msgs::msg::PoseStamped>("fr3_husky_controller/target_ee_pose", 10,std::bind(&FR3HuskyController::subtargetEEPoseCallback, this, std::placeholders::_1));
        target_base_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("fr3_husky_controller/cmd_vel", 10,std::bind(&FR3HuskyController::subtargetBaseVelCallback, this, std::placeholders::_1));
        joint_sub_           = node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states_raw", 10, std::bind(&FR3HuskyController::subJointStatesCallback, this, std::placeholders::_1));
        
        current_ee_pose_pub_    = node_->create_publisher<geometry_msgs::msg::PoseStamped>("fr3_husky_controller/ee_pose", 10);
        current_base_pose_pub_  = node_->create_publisher<geometry_msgs::msg::PoseStamped>("fr3_husky_controller/base_pose", 10);
        current_base_vel_pub_   = node_->create_publisher<geometry_msgs::msg::Twist>("fr3_husky_controller/base_vel", 10);
        joint_pub_              = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        base_vel_.setZero();
        base_vel_desired_.setZero();
        base_vel_init_.setZero();

        q_virtual_.setZero();
        q_virtual_desired_.setZero();
        q_virtual_init_.setZero();
        qdot_virtual_.setZero();
        qdot_virtual_desired_.setZero();
        qdot_virtual_init_.setZero();
        
        q_mani_.setZero();
        q_mani_desired_.setZero();
        q_mani_init_.setZero();
        qdot_mani_.setZero();
        qdot_mani_desired_.setZero();
        qdot_mani_init_.setZero();
        
        q_mobile_.setZero();
        q_mobile_desired_.setZero();
        q_mobile_init_.setZero();
        qdot_mobile_.setZero();
        qdot_mobile_init_.setZero();
        
        x_goal_.setIdentity();
        ee_name_ = "fr3_hand_tcp";
        link_ee_task_[ee_name_] = drc::TaskSpaceData::Zero();
        
        torque_mani_desired_.setZero();
        qdot_mobile_desired_.setZero();

        setDRCGains();

        std::ostringstream oss;
        oss.clear();
        oss << "\n=================================================================\n"
            << "=================================================================\n"
            << "URDF Joint Information: FR3Husky\n"
            << robot_data_->getVerbose()
            << "=================================================================\n"
            << "=================================================================";
        const std::string print_info = oss.str();
        RCLCPP_INFO(node->get_logger(), "%s%s%s", cblue, print_info.c_str(), creset);
    }

    void FR3HuskyController::starting()
    {
        current_ee_pose_pub_timer_   = node_->create_wall_timer(std::chrono::milliseconds(50),  std::bind(&FR3HuskyController::pubEEPoseCallback, this));
        current_base_pose_pub_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), std::bind(&FR3HuskyController::pubBasePoseCallback, this));
        current_base_vel_pub_timer_  = node_->create_wall_timer(std::chrono::milliseconds(50), std::bind(&FR3HuskyController::pubBaseVelCallback, this));
    }

    void FR3HuskyController::updateState(const MujocoRosSim::VecMap& pos_dict, 
                                         const MujocoRosSim::VecMap& vel_dict,
                                         const MujocoRosSim::VecMap& tau_ext_dict, 
                                         const MujocoRosSim::VecMap& sensors_dict, 
                                         double current_time)
    {
        current_time_ = current_time;

        // get manipulator joint
        for(size_t i=0; i<MANI_DOF; i++)
        {
            const std::string& name = "fr3_joint" + std::to_string(i+1);
            q_mani_(i) = pos_dict.at(name)(0);
            qdot_mani_(i) = vel_dict.at(name)(0);
        }

        // get mobile wheel joint
        q_mobile_(0) = pos_dict.at("front_left_wheel")(0);
        q_mobile_(1) = pos_dict.at("front_right_wheel")(0);
        qdot_mobile_(0) = vel_dict.at("front_left_wheel")(0);
        qdot_mobile_(1) = vel_dict.at("front_right_wheel")(0);

        // get virtual joint
        // using odometry for getting virtual joint (you can use SLAM instead)
        const Eigen::Affine2d base_pose_w = robot_data_->computeBasePose(q_mobile_, qdot_mobile_);
        q_virtual_ << base_pose_w.translation()(0), 
                      base_pose_w.translation()(1),
                      Eigen::Rotation2Dd(base_pose_w.linear()).angle();
        base_vel_  = robot_data_->computeBaseVel(q_mobile_, qdot_mobile_);
        qdot_virtual_.head(2) = base_pose_w.linear() * base_vel_.head(2);
        qdot_virtual_(2) = base_vel_(2);

        if(!robot_data_->updateState(q_virtual_, q_mobile_, q_mani_, qdot_virtual_, qdot_mobile_, qdot_mani_))
        {
            RCLCPP_ERROR(node_->get_logger(), "%sFailed to update robot state.%s", cred, creset);
        }

        // get task link states
        for (auto& [link_name, task] : link_ee_task_)
        {
            task.x = robot_data_->getPose(link_name);
            task.xdot = robot_data_->getVelocity(link_name);
        }
    }

    void FR3HuskyController::updateRGBDImage(const MujocoRosSim::ImageCVMap& images)
    {

    }

    void FR3HuskyController::compute()
    {
        if(is_mode_changed_)
        {
            is_mode_changed_ = false;

            control_start_time_ = current_time_;

            base_vel_init_= base_vel_;
            base_vel_desired_.setZero();

            q_virtual_init_ = q_virtual_;
            qdot_virtual_init_ = qdot_virtual_;
            q_virtual_desired_ = q_virtual_init_;
            qdot_virtual_desired_.setZero();

            q_mani_init_ = q_mani_;
            qdot_mani_init_ = qdot_mani_;
            q_mani_desired_ = q_mani_init_;
            qdot_mani_desired_.setZero();
            
            q_mobile_init_ = q_mobile_;
            qdot_mobile_init_ = qdot_mobile_;
            q_mobile_desired_ = q_mobile_init_;
            qdot_mobile_desired_.setZero();

            x_goal_ = link_ee_task_[ee_name_].x;

            for (auto& [link_name, task] : link_ee_task_)
            {
                task.setInit();
                task.xdot.setZero();
                task.setDesired();
                task.control_start_time = current_time_;
            }
        }

        if(mode_ == "CLIK" || mode_ == "QPIK" || mode_ == "OSF" || mode_ == "QPID")
        {
            if(is_goal_pose_changed_)
            {
                control_start_time_ = current_time_;
                
                link_ee_task_[ee_name_].setInit();
                link_ee_task_[ee_name_].xdot.setZero();
                link_ee_task_[ee_name_].x_desired =  x_goal_;
                link_ee_task_[ee_name_].control_start_time = current_time_;

                is_goal_pose_changed_ = false;
            }
        }
        
        if(mode_ == "HOME")
        {
            ManiVec q_mani_target;
            q_mani_target << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
            torque_mani_desired_ = robot_controller_->moveManipulatorJointTorqueCubic(q_mani_target,
                                                                                      ManiVec::Zero(),
                                                                                      q_mani_init_,
                                                                                      qdot_mani_init_,
                                                                                      current_time_,
                                                                                      control_start_time_,
                                                                                      4.0,
                                                                                      false);
            qdot_mobile_desired_.setZero();
        }
        else if(mode_ == "CLIK")
        {
            MobiVec qdot_mobile_desired;
            ManiVec qdot_mani_desired;
            if(robot_controller_->CLIKCubic(link_ee_task_, current_time_, 4.0, qdot_mobile_desired, qdot_mani_desired))
            {
                qdot_mobile_desired_ = qdot_mobile_desired;
                qdot_mani_desired_ = qdot_mani_desired;
                q_mani_desired_ += dt_ * qdot_mani_desired_;
                torque_mani_desired_ = robot_controller_->moveManipulatorJointTorqueStep(q_mani_desired_, qdot_mani_desired_, false);
            }
            else
            {
                torque_mani_desired_ = robot_data_->getGravity().segment(robot_data_->getJointIndex().mani_start, MANI_DOF);
                qdot_mobile_desired_.setZero();
            }
        }
        else if(mode_ == "QPIK")
        {
            MobiVec qdot_mobile_desired;
            ManiVec qdot_mani_desired;
            if(robot_controller_->QPIKCubic(link_ee_task_, current_time_, 4.0, qdot_mobile_desired, qdot_mani_desired))
            {
                qdot_mobile_desired_ = qdot_mobile_desired;
                qdot_mani_desired_ = qdot_mani_desired;
                q_mani_desired_ += dt_ * qdot_mani_desired_;
                torque_mani_desired_ = robot_controller_->moveManipulatorJointTorqueStep(q_mani_desired_, qdot_mani_desired_, false);
            }
            else
            {
                torque_mani_desired_ = robot_data_->getGravity().segment(robot_data_->getJointIndex().mani_start, MANI_DOF);
                qdot_mobile_desired_.setZero();
            }
        }
        else if(mode_ == "OSF")
        {
            MobiVec qddot_mobile_desired;
            ManiVec torque_mani_desired;

            ManiVec target_q;
            target_q << 0.0, 0.0, 0.0, -M_PI/2., 0.0, M_PI/2., M_PI / 4.;
            AactuatorVec null_torque_desired;
            null_torque_desired.setZero();
            null_torque_desired.segment(robot_data_->getActuatorIndex().mani_start, MANI_DOF) =
                robot_controller_->moveManipulatorJointTorqueStep(target_q, ManiVec::Zero(), false);
            
            if(robot_controller_->OSFCubic(link_ee_task_, current_time_, 4.0, qddot_mobile_desired, torque_mani_desired, null_torque_desired))
            {
                torque_mani_desired_ = torque_mani_desired;
                qdot_mobile_desired_ += dt_ * qddot_mobile_desired;
            }
            else
            {
                torque_mani_desired_ = robot_data_->getGravity().segment(robot_data_->getJointIndex().mani_start, MANI_DOF);
                qdot_mobile_desired_.setZero();
            }
        }
        else if(mode_ == "QPID")
        {
            MobiVec qddot_mobile_desired;
            ManiVec torque_mani_desired;
            if(robot_controller_->QPIDCubic(link_ee_task_, current_time_, 4.0, qddot_mobile_desired,torque_mani_desired, false))
            {
                torque_mani_desired_ = torque_mani_desired;
                qdot_mobile_desired_ += dt_ * qddot_mobile_desired;
            }
            else
            {
                torque_mani_desired_ = robot_data_->getGravity().segment(robot_data_->getJointIndex().mani_start, MANI_DOF);
                qdot_mobile_desired_.setZero();
            }
        }
        else if(mode_ == "Gravity_compensattion_W_QPID")
        {
            MobiVec qddot_mobile_desired;
            ManiVec torque_mani_desired;
            link_ee_task_[ee_name_].xddot_desired.setZero();

            if(robot_controller_->QPID(link_ee_task_, qddot_mobile_desired, torque_mani_desired))
            {
                torque_mani_desired_ = torque_mani_desired;
                qdot_mobile_desired_ += dt_ * qddot_mobile_desired;
            }
            else
            {
                torque_mani_desired_ = robot_data_->getGravity().segment(robot_data_->getJointIndex().mani_start, MANI_DOF);
                qdot_mobile_desired_.setZero();
            }
        }
        else if(mode_ == "Base Velocity Tracking")
        {
            torque_mani_desired_ = robot_controller_->moveManipulatorJointTorqueStep(q_mani_init_, ManiVec::Zero(), false);
            qdot_mobile_desired_ = robot_controller_->MobileVelocityCommand(base_vel_desired_);
        }
        else
        {
            torque_mani_desired_ = robot_data_->getGravity().segment(robot_data_->getJointIndex().mani_start, MANI_DOF);
            qdot_mobile_desired_.setZero();
        }
    }

    MujocoRosSim::CtrlInputMap FR3HuskyController::getCtrlInput() const
    {
        MujocoRosSim::CtrlInputMap ctrl_dict;
        ctrl_dict["left_wheel"] = qdot_mobile_desired_(0);
        ctrl_dict["right_wheel"] = qdot_mobile_desired_(1);
        for(size_t i=0; i<MANI_DOF; i++)
        {
            const std::string name = "fr3_joint" + std::to_string(i+1);
            ctrl_dict[name] = torque_mani_desired_(i);
        }

        return ctrl_dict;
    }

    void FR3HuskyController::setMode(const std::string& mode)
    {
        is_mode_changed_ = true;
        mode_ = mode;
        RCLCPP_INFO(node_->get_logger(), "\033[34m Mode changed: %s\033[0m", mode.c_str());
    }

    void FR3HuskyController::keyCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Key input received: %d", msg->data);
        if(msg->data == 1)      setMode("HOME");
        else if(msg->data == 2) setMode("CLIK");
        else if(msg->data == 3) setMode("QPIK");
        else if(msg->data == 4) setMode("OSF");
        else if(msg->data == 5) setMode("QPID");
        else if(msg->data == 6) setMode("Gravity_compensattion_W_QPID");
        else if(msg->data == 7) setMode("Base Velocity Tracking");
        else                    setMode("NONE");
    }

    void FR3HuskyController::subtargetEEPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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

        x_goal_.linear() = quat.toRotationMatrix();
        x_goal_.translation() << msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z;
        is_goal_pose_changed_ = true;
    }

    void FR3HuskyController::subtargetBaseVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Target base velocity received: linear=(%.3f, %.3f, %.3f), "
                    "angular=(%.3f, %.3f, %.3f)",
                    msg->linear.x, msg->linear.y, msg->linear.z,
                    msg->angular.x, msg->angular.y, msg->angular.z);

        base_vel_desired_.head(2) << msg->linear.x, msg->linear.y;
        base_vel_desired_(2) = msg->angular.z;

        auto& ee_task = link_ee_task_[ee_name_];
        ee_task.xdot_desired.head<3>() << msg->linear.x, msg->linear.y, msg->linear.z;
        // ee_task.xdot_desired.tail<3>() << msg->angular.x, msg->angular.y, msg->angular.z;
    }

    void FR3HuskyController::subJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header = msg->header;
        joint_msg.name = msg->name;
        joint_msg.position = msg->position;
        joint_msg.velocity = msg->velocity;
        joint_msg.effort = msg->effort;

        const std::vector<std::string> virtual_joints = {"v_x_joint", "v_y_joint", "v_t_joint"};
        std::vector<double> virtual_pos_vec(q_virtual_.data(), q_virtual_.data() + q_virtual_.size());
        std::vector<double> virtual_vel_vec(qdot_virtual_.data(), qdot_virtual_.data() + qdot_virtual_.size());
        std::vector<double> virtual_eff_vec{0,0,0};

        joint_msg.name.insert(joint_msg.name.end(), virtual_joints.begin(), virtual_joints.end());
        joint_msg.position.insert(joint_msg.position.end(), virtual_pos_vec.begin(), virtual_pos_vec.end());
        joint_msg.velocity.insert(joint_msg.velocity.end(), virtual_vel_vec.begin(), virtual_vel_vec.end());
        joint_msg.effort.insert(joint_msg.effort.end(), virtual_eff_vec.begin(), virtual_eff_vec.end());

        joint_pub_->publish(joint_msg);
    }

    void FR3HuskyController::pubEEPoseCallback()
    {
        auto ee_pose_msg = geometry_msgs::msg::PoseStamped();
        ee_pose_msg.header.frame_id = "world";
        ee_pose_msg.header.stamp = node_->now();

        ee_pose_msg.pose.position.x = link_ee_task_[ee_name_].x.translation()(0);
        ee_pose_msg.pose.position.y = link_ee_task_[ee_name_].x.translation()(1);
        ee_pose_msg.pose.position.z = link_ee_task_[ee_name_].x.translation()(2);

        Eigen::Quaterniond q(link_ee_task_[ee_name_].x.rotation());
        ee_pose_msg.pose.orientation.x = q.x();
        ee_pose_msg.pose.orientation.y = q.y();
        ee_pose_msg.pose.orientation.z = q.z();
        ee_pose_msg.pose.orientation.w = q.w();
        
        current_ee_pose_pub_->publish(ee_pose_msg);
    }

    void FR3HuskyController::pubBasePoseCallback()
    {
        auto base_pose_msg = geometry_msgs::msg::PoseStamped();
        base_pose_msg.header.frame_id = "world";
        base_pose_msg.header.stamp = node_->now();

        base_pose_msg.pose.position.x = q_virtual_(0);
        base_pose_msg.pose.position.y = q_virtual_(1);
        base_pose_msg.pose.position.z = 0;

        Eigen::Quaterniond quat(Eigen::AngleAxisd(q_virtual_(2), Eigen::Vector3d::UnitZ()));
        base_pose_msg.pose.orientation.x = quat.x();
        base_pose_msg.pose.orientation.y = quat.y();
        base_pose_msg.pose.orientation.z = quat.z();
        base_pose_msg.pose.orientation.w = quat.w();
        
        current_base_pose_pub_->publish(base_pose_msg);
    }

    void FR3HuskyController::pubBaseVelCallback()
    {
        auto base_vel_msg = geometry_msgs::msg::Twist();
        base_vel_msg.linear.x = base_vel_(0);
        base_vel_msg.linear.y = base_vel_(1);
        base_vel_msg.angular.z = base_vel_(2);
        
        current_base_vel_pub_->publish(base_vel_msg);
    }

    bool FR3HuskyController::setDRCGains()
    {
        const std::string joint = "dyros_robot_controller.manipulator_joint_gains.";
        const std::string task  = "dyros_robot_controller.task_gains.";
        const std::string qpik  = "dyros_robot_controller.QPIK_weight.";
        const std::string qpid  = "dyros_robot_controller.QPID_weight.";

        std::vector<double> joint_kp_vec = node_->declare_parameter<std::vector<double>>(joint + "kp", std::vector<double>(MANI_DOF, 1.0));
        std::vector<double> joint_kv_vec = node_->declare_parameter<std::vector<double>>(joint + "kv", std::vector<double>(MANI_DOF, 1.0));

        std::vector<double> task_ik_kp_vec = node_->declare_parameter<std::vector<double>>(task + "ik.kp", std::vector<double>(TASK_DOF, 1.0));
        std::vector<double> task_id_kp_vec = node_->declare_parameter<std::vector<double>>(task + "id.kp", std::vector<double>(TASK_DOF, 1.0));
        std::vector<double> task_id_kv_vec = node_->declare_parameter<std::vector<double>>(task + "id.kd", std::vector<double>(TASK_DOF, 1.0));

        std::vector<double> qpik_tracking_vec         = node_->declare_parameter<std::vector<double>>(qpik + "tracking.weights",               std::vector<double>(TASK_DOF, 1.0));
        std::vector<double> qpik_mani_vel_damping_vec = node_->declare_parameter<std::vector<double>>(qpik + "joint.velocity.manipulator",     std::vector<double>(MANI_DOF, 1.0));
        std::vector<double> qpik_mani_acc_damping_vec = node_->declare_parameter<std::vector<double>>(qpik + "joint.acceleration.manipulator", std::vector<double>(MANI_DOF, 1.0));
        std::vector<double> qpik_mobi_vel_damping_vec = node_->declare_parameter<std::vector<double>>(qpik + "joint.velocity.mobile",          std::vector<double>(VIRTUAL_DOF, 1.0));
        std::vector<double> qpik_mobi_acc_damping_vec = node_->declare_parameter<std::vector<double>>(qpik + "joint.acceleration.mobile",      std::vector<double>(VIRTUAL_DOF, 1.0));

        std::vector<double> qpid_tracking_vec         = node_->declare_parameter<std::vector<double>>(qpid + "tracking.weights",               std::vector<double>(TASK_DOF, 1.0));
        std::vector<double> qpid_mani_vel_damping_vec = node_->declare_parameter<std::vector<double>>(qpid + "joint.velocity.manipulator",     std::vector<double>(MANI_DOF, 1.0));
        std::vector<double> qpid_mani_acc_damping_vec = node_->declare_parameter<std::vector<double>>(qpid + "joint.acceleration.manipulator", std::vector<double>(MANI_DOF, 1.0));
        std::vector<double> qpid_mobi_vel_damping_vec = node_->declare_parameter<std::vector<double>>(qpid + "joint.velocity.mobile",          std::vector<double>(VIRTUAL_DOF, 1.0));
        std::vector<double> qpid_mobi_acc_damping_vec = node_->declare_parameter<std::vector<double>>(qpid + "joint.acceleration.mobile",      std::vector<double>(VIRTUAL_DOF, 1.0));

        auto check_vector_size = [this](const std::string & name, size_t expected, size_t got) -> bool
        {
            if (got != expected)
            {
                RCLCPP_ERROR(node_->get_logger(), "%sParameter '%s' expected %zu values, got %zu.%s", cred, name.c_str(), expected, got, creset);
                return false;
            }
            return true;
        };

        // manipulator joint gains
        if (!check_vector_size(joint + "kp", MANI_DOF, joint_kp_vec.size())) return false;
        if (!check_vector_size(joint + "kv", MANI_DOF, joint_kv_vec.size())) return false;

        // task-space gains
        if (!check_vector_size(task + "ik.kp", TASK_DOF, task_ik_kp_vec.size())) return false;
        if (!check_vector_size(task + "id.kp", TASK_DOF, task_id_kp_vec.size())) return false;
        if (!check_vector_size(task + "id.kd", TASK_DOF, task_id_kv_vec.size())) return false;

        // QPIK weights
        if (!check_vector_size(qpik + "tracking.weights",               TASK_DOF,    qpik_tracking_vec.size()))         return false;
        if (!check_vector_size(qpik + "joint.velocity.manipulator",     MANI_DOF,    qpik_mani_vel_damping_vec.size())) return false;
        if (!check_vector_size(qpik + "joint.acceleration.manipulator", MANI_DOF,    qpik_mani_acc_damping_vec.size())) return false;
        if (!check_vector_size(qpik + "joint.velocity.mobile",          VIRTUAL_DOF, qpik_mobi_vel_damping_vec.size())) return false;
        if (!check_vector_size(qpik + "joint.acceleration.mobile",      VIRTUAL_DOF, qpik_mobi_acc_damping_vec.size())) return false;

        // QPID weights
        if (!check_vector_size(qpid + "tracking.weights",               TASK_DOF,    qpid_tracking_vec.size()))         return false;
        if (!check_vector_size(qpid + "joint.velocity.manipulator",     MANI_DOF,    qpid_mani_vel_damping_vec.size())) return false;
        if (!check_vector_size(qpid + "joint.acceleration.manipulator", MANI_DOF,    qpid_mani_acc_damping_vec.size())) return false;
        if (!check_vector_size(qpid + "joint.velocity.mobile",          VIRTUAL_DOF, qpid_mobi_vel_damping_vec.size())) return false;
        if (!check_vector_size(qpid + "joint.acceleration.mobile",      VIRTUAL_DOF, qpid_mobi_acc_damping_vec.size())) return false;

        joint_kp_ = Eigen::Map<const Eigen::VectorXd>(joint_kp_vec.data(), joint_kp_vec.size());
        joint_kv_ = Eigen::Map<const Eigen::VectorXd>(joint_kv_vec.data(), joint_kv_vec.size());

        task_ik_kp_ = Eigen::Map<const Eigen::VectorXd>(task_ik_kp_vec.data(), task_ik_kp_vec.size());
        task_id_kp_ = Eigen::Map<const Eigen::VectorXd>(task_id_kp_vec.data(), task_id_kp_vec.size());
        task_id_kv_ = Eigen::Map<const Eigen::VectorXd>(task_id_kv_vec.data(), task_id_kv_vec.size());
 
        qpik_tracking_         = Eigen::Map<const Eigen::VectorXd>(qpik_tracking_vec.data(),         qpik_tracking_vec.size());
        qpik_mani_vel_damping_ = Eigen::Map<const Eigen::VectorXd>(qpik_mani_vel_damping_vec.data(), qpik_mani_vel_damping_vec.size());
        qpik_mani_acc_damping_ = Eigen::Map<const Eigen::VectorXd>(qpik_mani_acc_damping_vec.data(), qpik_mani_acc_damping_vec.size());
        qpik_mobi_vel_damping_ = Eigen::Map<const Eigen::VectorXd>(qpik_mobi_vel_damping_vec.data(), qpik_mobi_vel_damping_vec.size());
        qpik_mobi_acc_damping_ = Eigen::Map<const Eigen::VectorXd>(qpik_mobi_acc_damping_vec.data(), qpik_mobi_acc_damping_vec.size());
        
        qpid_tracking_        = Eigen::Map<const Eigen::VectorXd>(qpid_tracking_vec.data(),          qpid_tracking_vec.size());
        qpid_mani_vel_damping_ = Eigen::Map<const Eigen::VectorXd>(qpid_mani_vel_damping_vec.data(), qpid_mani_vel_damping_vec.size());
        qpid_mani_acc_damping_ = Eigen::Map<const Eigen::VectorXd>(qpid_mani_acc_damping_vec.data(), qpid_mani_acc_damping_vec.size());
        qpid_mobi_vel_damping_ = Eigen::Map<const Eigen::VectorXd>(qpid_mobi_vel_damping_vec.data(), qpid_mobi_vel_damping_vec.size());
        qpid_mobi_acc_damping_ = Eigen::Map<const Eigen::VectorXd>(qpid_mobi_acc_damping_vec.data(), qpid_mobi_acc_damping_vec.size());

        std::ostringstream oss;
        oss << "dyros robot controller gains" << "\n"
            << "\tmanipulator_joint_gains" << "\n"
            << "\t\tKp: " << joint_kp_.transpose() << "\n"
            << "\t\tKv: " << joint_kv_.transpose() << "\n"
            << "\ttask_gains" << "\n"
            << "\t\tik.Kp: " << task_ik_kp_.transpose() << "\n"
            << "\t\tid.Kp: " << task_id_kp_.transpose() << "\n"
            << "\t\tid.Kv: " << task_id_kv_.transpose() << "\n"
            << "\tQPIK_weight" << "\n"
            << "\t\ttracking.weights: " << qpik_tracking_.transpose() << "\n"
            << "\t\tjoint.velocity.manipulator: " << qpik_mani_vel_damping_.transpose() << "\n"
            << "\t\tjoint.acceleration.manipulator: " << qpik_mani_acc_damping_.transpose() << "\n"
            << "\t\tjoint.velocity.mobile: " << qpik_mobi_vel_damping_.transpose() << "\n"
            << "\t\tjoint.acceleration.mobile: " << qpik_mobi_acc_damping_.transpose() << "\n"
            << "\tQPID_weight" << "\n"
            << "\t\ttracking.weights: " << qpid_tracking_.transpose() << "\n"
            << "\t\tjoint.velocity.manipulator: " << qpid_mani_vel_damping_.transpose() << "\n"
            << "\t\tjoint.acceleration.manipulator: " << qpid_mani_acc_damping_.transpose() << "\n"
            << "\t\tjoint.velocity.mobile: " << qpid_mobi_vel_damping_.transpose() << "\n"
            << "\t\tjoint.acceleration.mobile: " << qpid_mobi_acc_damping_.transpose();
        RCLCPP_INFO(node_->get_logger(), "%s%s%s", cblue, oss.str().c_str(), creset);

        robot_controller_->setManipulatorJointGain(joint_kp_, joint_kv_);
        robot_controller_->setIKGain(task_ik_kp_);
        robot_controller_->setIDGain(task_id_kp_, task_id_kv_);
        robot_controller_->setQPIKGain(qpik_tracking_, qpik_mani_vel_damping_, qpik_mani_acc_damping_, qpik_mobi_vel_damping_, qpik_mobi_acc_damping_);
        robot_controller_->setQPIDGain(qpid_tracking_, qpid_mani_vel_damping_, qpid_mani_acc_damping_, qpid_mobi_vel_damping_, qpid_mobi_acc_damping_);
        return true;
    }

    /* register with the global registry */
    PLUGINLIB_EXPORT_CLASS(FR3Husky::FR3HuskyController, MujocoRosSim::ControllerInterface)
} // namespace FR3Husky
