#include "dyros_robot_menagerie/fr3_xls/controller.h"

namespace FR3XLS
{
    void FR3XLSController::configure(const rclcpp::Node::SharedPtr& node)
    {
        MujocoRosSim::ControllerInterface::configure(node);
        const double dt = 0.001;

        robot_data_ = std::make_shared<FR3XLSRobotData>(dt);
        dt_ = robot_data_->getDt();
        robot_controller_ = std::make_unique<drc::MobileManipulator::RobotController>(robot_data_);
        
        key_sub_             = node_->create_subscription<std_msgs::msg::Int32>("fr3_xls_controller/mode_input", 10,std::bind(&FR3XLSController::keyCallback, this, std::placeholders::_1));
        target_ee_pose_sub_  = node_->create_subscription<geometry_msgs::msg::PoseStamped>("fr3_xls_controller/target_ee_pose", 10,std::bind(&FR3XLSController::subtargetEEPoseCallback, this, std::placeholders::_1));
        target_base_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("fr3_xls_controller/cmd_vel", 10,std::bind(&FR3XLSController::subtargetBaseVelCallback, this, std::placeholders::_1));
        joint_sub_           = node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states_raw", 10, std::bind(&FR3XLSController::subJointStatesCallback, this, std::placeholders::_1));
        
        current_ee_pose_pub_    = node_->create_publisher<geometry_msgs::msg::PoseStamped>("fr3_xls_controller/ee_pose", 10);
        current_base_pose_pub_  = node_->create_publisher<geometry_msgs::msg::PoseStamped>("fr3_xls_controller/base_pose", 10);
        current_base_vel_pub_   = node_->create_publisher<geometry_msgs::msg::Twist>("fr3_xls_controller/base_vel", 10);
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
        link_ee_name_ = robot_data_->getEEName();
        link_ee_task_[link_ee_name_] = drc::TaskSpaceData::Zero();
        
        x_goal_.setIdentity();
        
        torque_mani_desired_.setZero();
        qdot_mobile_desired_.setZero();

        std::vector<double> mani_joint_kp_vec    = node->declare_parameter<std::vector<double>>("manipulator_joint_gains.kp", {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0});
        std::vector<double> mani_joint_kv_vec    = node->declare_parameter<std::vector<double>>("manipulator_joint_gains.kv", {30.0,  30.0,  30.0,  30.0,  10.0,  10.0,  5.0});
        std::vector<double> task_kp_vec          = node->declare_parameter<std::vector<double>>("task_gains.kp",              {100.0, 100.0, 100.0, 100.0, 100.0, 100.0});
        std::vector<double> task_kv_vec          = node->declare_parameter<std::vector<double>>("task_gains.kv",              {20.0,  20.0,  20.0,  20.0,  20.0,  20.0});
        std::vector<double> qpik_tracking_vec    = node->declare_parameter<std::vector<double>>("QPIK_gains.tracking",        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        std::vector<double> qpik_damping_vec     = node->declare_parameter<std::vector<double>>("QPIK_gains.damping",         {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        std::vector<double> qpid_tracking_vec    = node->declare_parameter<std::vector<double>>("QPID_gains.tracking",        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        std::vector<double> qpid_vel_damping_vec = node->declare_parameter<std::vector<double>>("QPID_gains.vel_damping",     {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
        std::vector<double> qpid_acc_damping_vec = node->declare_parameter<std::vector<double>>("QPID_gains.acc_damping",     {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
        
        mani_joint_kp_                     = Eigen::Map<Eigen::VectorXd>(mani_joint_kp_vec.data(),    mani_joint_kp_vec.size());
        mani_joint_kv_                     = Eigen::Map<Eigen::VectorXd>(mani_joint_kv_vec.data(),    mani_joint_kv_vec.size());
        link_task_kp_[link_ee_name_]       = Eigen::Map<Eigen::VectorXd>(task_kp_vec.data(),          task_kp_vec.size());
        link_task_kv_[link_ee_name_]       = Eigen::Map<Eigen::VectorXd>(task_kv_vec.data(),          task_kv_vec.size());
        link_qpik_tracking_[link_ee_name_] = Eigen::Map<Eigen::VectorXd>(qpik_tracking_vec.data(),    qpik_tracking_vec.size());
        qpik_damping_                      = Eigen::Map<Eigen::VectorXd>(qpik_damping_vec.data(),     qpik_damping_vec.size());
        link_qpid_tracking_[link_ee_name_] = Eigen::Map<Eigen::VectorXd>(qpid_tracking_vec.data(),    qpid_tracking_vec.size());
        qpid_vel_damping_                  = Eigen::Map<Eigen::VectorXd>(qpid_vel_damping_vec.data(), qpid_vel_damping_vec.size());
        qpid_acc_damping_                  = Eigen::Map<Eigen::VectorXd>(qpid_acc_damping_vec.data(), qpid_acc_damping_vec.size());

        if (mani_joint_kp_.size()                     != MANI_DOF)     RCLCPP_WARN(node->get_logger(), "manipulator_joint_gains.kp size mismatch (expected 7)");
        if (mani_joint_kv_.size()                     != MANI_DOF)     RCLCPP_WARN(node->get_logger(), "manipulator_joint_gains.kv size mismatch (expected 7)");
        if (link_task_kp_[link_ee_name_].size()       != TASK_DOF)     RCLCPP_WARN(node->get_logger(), "task_gains.kp.size mismatch (expected 6)");
        if (link_task_kv_[link_ee_name_].size()       != TASK_DOF)     RCLCPP_WARN(node->get_logger(), "task_gains.kv.size mismatch (expected 6)");
        if (link_qpik_tracking_[link_ee_name_].size() != TASK_DOF)     RCLCPP_WARN(node->get_logger(), "QPIK_gains.tracking.size mismatch (expected 6)");
        if (qpik_damping_.size()                      != ACTUATOR_DOF) RCLCPP_WARN(node->get_logger(), "QPIK_gains.damping size mismatch (expected 11)");
        if (link_qpid_tracking_[link_ee_name_].size() != TASK_DOF)     RCLCPP_WARN(node->get_logger(), "QPID_gains.tracking.size mismatch (expected 6)");
        if (qpid_vel_damping_.size()                  != ACTUATOR_DOF) RCLCPP_WARN(node->get_logger(), "QPID_gains.vel_damping size mismatch (expected 11)");
        if (qpid_acc_damping_.size()                  != ACTUATOR_DOF) RCLCPP_WARN(node->get_logger(), "QPID_gains.acc_damping size mismatch (expected 11)");

        robot_controller_->setManipulatorJointGain(mani_joint_kp_, mani_joint_kv_);
        robot_controller_->setTaskGain(link_task_kp_, link_task_kv_);
        robot_controller_->setQPIKGain(link_qpik_tracking_, qpik_damping_);
        robot_controller_->setQPIDGain(link_qpid_tracking_, qpid_vel_damping_, qpid_acc_damping_);

        std::ostringstream oss;
        oss << "\n=================================================================\n"
            << "=================================================================\n"
            << "URDF Joint Information: FR3XLS\n"
            << robot_data_->getVerbose()
            << "=================================================================\n"
            << "=================================================================";
        const std::string print_info = oss.str();
        RCLCPP_INFO(node->get_logger(), "%s%s%s", cblue, print_info.c_str(), creset);
    }

    void FR3XLSController::starting()
    {
        current_ee_pose_pub_timer_   = node_->create_wall_timer(std::chrono::milliseconds(50),  std::bind(&FR3XLSController::pubEEPoseCallback, this));
        current_base_pose_pub_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&FR3XLSController::pubBasePoseCallback, this));
        current_base_vel_pub_timer_  = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&FR3XLSController::pubBaseVelCallback, this));
    }

    void FR3XLSController::updateState(const MujocoRosSim::VecMap& pos_dict, 
                                         const MujocoRosSim::VecMap& vel_dict,
                                         const MujocoRosSim::VecMap& tau_ext_dict, 
                                         const MujocoRosSim::VecMap& sensors_dict, 
                                         double current_time)
    {
        current_time_ = current_time;
        
        // get virtual joint
        q_virtual_.head(2) = sensors_dict.at("position_sensor").head(2);
        Quaterniond quat(sensors_dict.at("orientation_sensor")(0),
                         sensors_dict.at("orientation_sensor")(1),
                         sensors_dict.at("orientation_sensor")(2),
                         sensors_dict.at("orientation_sensor")(3));
        Vector3d euler_rpy = DyrosMath::rot2Euler(quat.toRotationMatrix());
        q_virtual_(2) = euler_rpy(2);

        qdot_virtual_.head(2) = sensors_dict.at("linear_velocity_sensor").head(2);
        qdot_virtual_(2) = sensors_dict.at("angular_velocity_sensor")(2);
        
        // get mobile base velocity
        Matrix2d rot_base2world;
        rot_base2world << cos(q_virtual_(2)), sin(q_virtual_(2)),
                         -sin(q_virtual_(2)), cos(q_virtual_(2));
        base_vel_.head(2) = rot_base2world * qdot_virtual_.head(2);
        base_vel_(2) = qdot_virtual_(2);

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
        q_mobile_(2) = pos_dict.at("rear_left_wheel")(0);
        q_mobile_(3) = pos_dict.at("rear_right_wheel")(0);
        qdot_mobile_(0) = vel_dict.at("front_left_wheel")(0);
        qdot_mobile_(1) = vel_dict.at("front_right_wheel")(0);
        qdot_mobile_(2) = vel_dict.at("rear_left_wheel")(0);
        qdot_mobile_(3) = vel_dict.at("rear_right_wheel")(0);

        if(!robot_data_->updateState(q_virtual_, q_mobile_, q_mani_, qdot_virtual_, qdot_mobile_, qdot_mani_))
        {
            RCLCPP_ERROR(node_->get_logger(), "%sFailed to update robot state.%s", cred, creset);
        }

        // get ee
        link_ee_task_[link_ee_name_].x    = robot_data_->getPose();
        link_ee_task_[link_ee_name_].xdot = robot_data_->getVelocity();
    }

    void FR3XLSController::updateRGBDImage(const MujocoRosSim::ImageCVMap& images)
    {

    }

    void FR3XLSController::compute()
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

            x_goal_ = link_ee_task_[link_ee_name_].x;

            link_ee_task_[link_ee_name_].setInit();
            link_ee_task_[link_ee_name_].setDesired();
            link_ee_task_[link_ee_name_].xdot.setZero();
        }

        if(mode_ == "QPIK" || mode_ == "QPID")
        {
            if(is_goal_pose_changed_)
            {
                control_start_time_ = current_time_;
                
                link_ee_task_[link_ee_name_].setInit();
                link_ee_task_[link_ee_name_].xdot.setZero();
                link_ee_task_[link_ee_name_].x_desired =  x_goal_;

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
        else if(mode_ == "QPIK")
        {
            VectorXd qdot_mobile_desired, qdot_mani_desired;
            robot_controller_->QPIKCubic(link_ee_task_, current_time_, control_start_time_, 4.0, qdot_mobile_desired, qdot_mani_desired);
            qdot_mobile_desired_ = qdot_mobile_desired;
            qdot_mani_desired_ = qdot_mani_desired;
            q_mani_desired_ += dt_ * qdot_mani_desired_;
            torque_mani_desired_ = robot_controller_->moveManipulatorJointTorqueStep(q_mani_desired_, qdot_mani_desired_, false);
        }
        else if(mode_ == "QPID")
        {
            VectorXd qddot_mobile_desired,torque_mani_desired;
            robot_controller_->QPIDCubic(link_ee_task_, current_time_, control_start_time_, 4.0, qddot_mobile_desired,torque_mani_desired);
            torque_mani_desired_ = torque_mani_desired;
            qdot_mobile_desired_ += dt_ * qddot_mobile_desired;
        }
        else if(mode_ == "Gravity_compensattion_W_QPID")
        {
            VectorXd qddot_mobile_desired,torque_mani_desired;
            link_ee_task_[link_ee_name_].xddot_desired.setZero();
            robot_controller_->QPID(link_ee_task_, qddot_mobile_desired, torque_mani_desired);
            torque_mani_desired_ = torque_mani_desired;
            qdot_mobile_desired_ += dt_ * qddot_mobile_desired;
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

    MujocoRosSim::CtrlInputMap FR3XLSController::getCtrlInput() const
    {
        MujocoRosSim::CtrlInputMap ctrl_dict;
        ctrl_dict["front_left_wheel"] = qdot_mobile_desired_(0);
        ctrl_dict["front_right_wheel"] = qdot_mobile_desired_(1);
        ctrl_dict["rear_left_wheel"] = qdot_mobile_desired_(2);
        ctrl_dict["rear_right_wheel"] = qdot_mobile_desired_(3);
        for(size_t i=0; i<MANI_DOF; i++)
        {
            const std::string name = "fr3_joint" + std::to_string(i+1);
            ctrl_dict[name] = torque_mani_desired_(i);
        }

        return ctrl_dict;
    }

    void FR3XLSController::setMode(const std::string& mode)
    {
        is_mode_changed_ = true;
        mode_ = mode;
        RCLCPP_INFO(node_->get_logger(), "\033[34m Mode changed: %s\033[0m", mode.c_str());
    }

    void FR3XLSController::keyCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Key input received: %d", msg->data);
        if(msg->data == 1)      setMode("HOME");
        else if(msg->data == 2) setMode("QPIK");
        else if(msg->data == 3) setMode("QPID");
        else if(msg->data == 4) setMode("Gravity_compensattion_W_QPID");
        else if(msg->data == 5) setMode("Base Velocity Tracking");
        else                    setMode("NONE");
    }

    void FR3XLSController::subtargetEEPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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

    void FR3XLSController::subtargetBaseVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Target base velocity received: linear=(%.3f, %.3f, %.3f), "
                    "angular=(%.3f, %.3f, %.3f)",
                    msg->linear.x, msg->linear.y, msg->linear.z,
                    msg->angular.x, msg->angular.y, msg->angular.z);

        base_vel_desired_.head(2) << msg->linear.x, msg->linear.y;
        base_vel_desired_(2) = msg->angular.z;
    }

    void FR3XLSController::subJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
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

    void FR3XLSController::pubEEPoseCallback()
    {
        auto ee_pose_msg = geometry_msgs::msg::PoseStamped();
        ee_pose_msg.header.frame_id = "world";
        ee_pose_msg.header.stamp = node_->now();

        ee_pose_msg.pose.position.x = link_ee_task_[link_ee_name_].x.translation()(0);
        ee_pose_msg.pose.position.y = link_ee_task_[link_ee_name_].x.translation()(1);
        ee_pose_msg.pose.position.z = link_ee_task_[link_ee_name_].x.translation()(2);

        Eigen::Quaterniond q(link_ee_task_[link_ee_name_].x.rotation());
        ee_pose_msg.pose.orientation.x = q.x();
        ee_pose_msg.pose.orientation.y = q.y();
        ee_pose_msg.pose.orientation.z = q.z();
        ee_pose_msg.pose.orientation.w = q.w();
        
        current_ee_pose_pub_->publish(ee_pose_msg);
    }

    void FR3XLSController::pubBasePoseCallback()
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

    void FR3XLSController::pubBaseVelCallback()
    {
        auto base_vel_msg = geometry_msgs::msg::Twist();
        base_vel_msg.linear.x = base_vel_(0);
        base_vel_msg.linear.y = base_vel_(1);
        base_vel_msg.angular.z = base_vel_(2);
        
        current_base_vel_pub_->publish(base_vel_msg);
    }

    /* register with the global registry */
    PLUGINLIB_EXPORT_CLASS(FR3XLS::FR3XLSController, MujocoRosSim::ControllerInterface)
} // namespace FR3XLS
