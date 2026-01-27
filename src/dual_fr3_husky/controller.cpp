#include "dyros_robot_menagerie/dual_fr3_husky/controller.h"

namespace DualFR3Husky
{
    void DualFR3HuskyController::configure(const rclcpp::Node::SharedPtr& node)
    {
        MujocoRosSim::ControllerInterface::configure(node);
        const double dt = 0.001;

        robot_data_ = std::make_shared<DualFR3HuskyRobotData>(dt);
        dt_ = robot_data_->getDt();
        robot_controller_ = std::make_unique<drc::MobileManipulator::RobotController>(robot_data_);
        
        key_sub_              = node_->create_subscription<std_msgs::msg::Int32>("dual_fr3_husky_controller/mode_input", 10,std::bind(&DualFR3HuskyController::keyCallback, this, std::placeholders::_1));
        target_l_ee_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("dual_fr3_husky_controller/target_l_ee_pose", 10,std::bind(&DualFR3HuskyController::subtargetLEEPoseCallback, this, std::placeholders::_1));
        target_r_ee_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("dual_fr3_husky_controller/target_r_ee_pose", 10,std::bind(&DualFR3HuskyController::subtargetREEPoseCallback, this, std::placeholders::_1));
        target_base_vel_sub_  = node_->create_subscription<geometry_msgs::msg::Twist>("dual_fr3_husky_controller/cmd_vel", 10,std::bind(&DualFR3HuskyController::subtargetBaseVelCallback, this, std::placeholders::_1));
        joint_sub_            = node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states_raw", 10, std::bind(&DualFR3HuskyController::subJointStatesCallback, this, std::placeholders::_1));
        
        current_l_ee_pose_pub_  = node_->create_publisher<geometry_msgs::msg::PoseStamped>("dual_fr3_husky_controller/l_ee_pose", 10);
        current_r_ee_pose_pub_  = node_->create_publisher<geometry_msgs::msg::PoseStamped>("dual_fr3_husky_controller/r_ee_pose", 10);
        current_base_pose_pub_  = node_->create_publisher<geometry_msgs::msg::PoseStamped>("dual_fr3_husky_controller/base_pose", 10);
        current_base_vel_pub_   = node_->create_publisher<geometry_msgs::msg::Twist>("dual_fr3_husky_controller/base_vel", 10);
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

        x_l_goal_.setIdentity();
        x_r_goal_.setIdentity();
        link_ee_name_l_ = robot_data_->getLEEName();
        link_ee_name_r_ = robot_data_->getREEName();
        link_ee_task_[link_ee_name_l_] = drc::TaskSpaceData::Zero();
        link_ee_task_[link_ee_name_r_] = drc::TaskSpaceData::Zero();
        
        torque_mani_desired_.setZero();
        qdot_mobile_desired_.setZero();


        std::vector<double> mani_joint_kp_vec    = node->declare_parameter<std::vector<double>>("manipulator_joint_gains.kp",   {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0,
                                                                                                                                 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0});
        std::vector<double> mani_joint_kv_vec    = node->declare_parameter<std::vector<double>>("manipulator_joint_gains.kv",   {30.0,  30.0,  30.0,  30.0,  10.0,  10.0,  5.0,
                                                                                                                                 30.0,  30.0,  30.0,  30.0,  10.0,  10.0,  5.0});
        std::vector<double> task_kp_l_vec        = node->declare_parameter<std::vector<double>>("task_gains.kp.left",           {100.0, 100.0, 100.0, 100.0, 100.0, 100.0});
        std::vector<double> task_kp_r_vec        = node->declare_parameter<std::vector<double>>("task_gains.kp.right",          {100.0, 100.0, 100.0, 100.0, 100.0, 100.0});
        std::vector<double> task_kv_l_vec        = node->declare_parameter<std::vector<double>>("task_gains.kv.left",           {20.0,  20.0,  20.0,  20.0,  20.0,  20.0});
        std::vector<double> task_kv_r_vec        = node->declare_parameter<std::vector<double>>("task_gains.kv.right",          {20.0,  20.0,  20.0,  20.0,  20.0,  20.0});
        std::vector<double> qpik_tracking_l_vec      = node->declare_parameter<std::vector<double>>("QPIK_gains.tracking.left",       {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        std::vector<double> qpik_tracking_r_vec      = node->declare_parameter<std::vector<double>>("QPIK_gains.tracking.right",      {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        std::vector<double> qpik_mani_damping_vec    = node->declare_parameter<std::vector<double>>("QPIK_gains.mani_damping",        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                                                                                                                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        std::vector<double> qpik_base_damping_vec    = node->declare_parameter<std::vector<double>>("QPIK_gains.base_damping",        {0.2, 0.2, 0.2});
        std::vector<double> qpid_tracking_l_vec      = node->declare_parameter<std::vector<double>>("QPID_gains.tracking.left",       {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        std::vector<double> qpid_tracking_r_vec      = node->declare_parameter<std::vector<double>>("QPID_gains.tracking.right",      {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        std::vector<double> qpid_mani_vel_damping_vec = node->declare_parameter<std::vector<double>>("QPID_gains.mani_vel_damping",   {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
                                                                                                                                0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
        std::vector<double> qpid_mani_acc_damping_vec = node->declare_parameter<std::vector<double>>("QPID_gains.mani_acc_damping",   {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                                                                                                                                0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
        std::vector<double> qpid_base_vel_damping_vec = node->declare_parameter<std::vector<double>>("QPID_gains.base_vel_damping",   {0.1, 0.1, 0.1});
        std::vector<double> qpid_base_acc_damping_vec = node->declare_parameter<std::vector<double>>("QPID_gains.base_acc_damping",   {0.01, 0.01, 0.01});
        
        mani_joint_kp_                       = Eigen::Map<Eigen::VectorXd>(mani_joint_kp_vec.data(),    mani_joint_kp_vec.size());
        mani_joint_kv_                       = Eigen::Map<Eigen::VectorXd>(mani_joint_kv_vec.data(),    mani_joint_kv_vec.size());
        link_task_kp_[link_ee_name_l_]       = Eigen::Map<Eigen::VectorXd>(task_kp_l_vec.data(),        task_kp_l_vec.size());
        link_task_kp_[link_ee_name_r_]       = Eigen::Map<Eigen::VectorXd>(task_kp_r_vec.data(),        task_kp_r_vec.size());
        link_task_kv_[link_ee_name_l_]       = Eigen::Map<Eigen::VectorXd>(task_kv_l_vec.data(),        task_kv_l_vec.size());
        link_task_kv_[link_ee_name_r_]       = Eigen::Map<Eigen::VectorXd>(task_kv_r_vec.data(),        task_kv_r_vec.size());
        link_qpik_tracking_[link_ee_name_l_] = Eigen::Map<Eigen::VectorXd>(qpik_tracking_l_vec.data(),      qpik_tracking_l_vec.size());
        link_qpik_tracking_[link_ee_name_r_] = Eigen::Map<Eigen::VectorXd>(qpik_tracking_r_vec.data(),      qpik_tracking_r_vec.size());
        qpik_mani_damping_                   = Eigen::Map<Eigen::VectorXd>(qpik_mani_damping_vec.data(),    qpik_mani_damping_vec.size());
        qpik_base_damping_                   = Eigen::Map<Eigen::Vector3d>(qpik_base_damping_vec.data());
        link_qpid_tracking_[link_ee_name_l_] = Eigen::Map<Eigen::VectorXd>(qpid_tracking_l_vec.data(),      qpid_tracking_l_vec.size());
        link_qpid_tracking_[link_ee_name_r_] = Eigen::Map<Eigen::VectorXd>(qpid_tracking_r_vec.data(),      qpid_tracking_r_vec.size());
        qpid_mani_vel_damping_               = Eigen::Map<Eigen::VectorXd>(qpid_mani_vel_damping_vec.data(), qpid_mani_vel_damping_vec.size());
        qpid_mani_acc_damping_               = Eigen::Map<Eigen::VectorXd>(qpid_mani_acc_damping_vec.data(), qpid_mani_acc_damping_vec.size());
        qpid_base_vel_damping_               = Eigen::Map<Eigen::Vector3d>(qpid_base_vel_damping_vec.data());
        qpid_base_acc_damping_               = Eigen::Map<Eigen::Vector3d>(qpid_base_acc_damping_vec.data());

        if (mani_joint_kp_.size()                       != MANI_DOF)     RCLCPP_WARN(node->get_logger(), "manipulator_joint_gains.kp size mismatch (expected 14)");
        if (mani_joint_kv_.size()                       != MANI_DOF)     RCLCPP_WARN(node->get_logger(), "manipulator_joint_gains.kv size mismatch (expected 14)");
        if (link_task_kp_[link_ee_name_l_].size()       != TASK_DOF)     RCLCPP_WARN(node->get_logger(), "task_gains.kp.left size mismatch (expected 6)");
        if (link_task_kp_[link_ee_name_r_].size()       != TASK_DOF)     RCLCPP_WARN(node->get_logger(), "task_gains.kp.right size mismatch (expected 6)");
        if (link_task_kv_[link_ee_name_l_].size()       != TASK_DOF)     RCLCPP_WARN(node->get_logger(), "task_gains.kv.left size mismatch (expected 6)");
        if (link_task_kv_[link_ee_name_r_].size()       != TASK_DOF)     RCLCPP_WARN(node->get_logger(), "task_gains.kv.right size mismatch (expected 6)");
        if (link_qpik_tracking_[link_ee_name_l_].size() != TASK_DOF)     RCLCPP_WARN(node->get_logger(), "QPIK_gains.tracking.left size mismatch (expected 6)");
        if (link_qpik_tracking_[link_ee_name_r_].size() != TASK_DOF)     RCLCPP_WARN(node->get_logger(), "QPIK_gains.tracking.right size mismatch (expected 6)");
        if (qpik_mani_damping_vec.size()                != MANI_DOF)     RCLCPP_WARN(node->get_logger(), "QPIK_gains.mani_damping size mismatch (expected 14)");
        if (qpik_base_damping_vec.size()                != 3)            RCLCPP_WARN(node->get_logger(), "QPIK_gains.base_damping size mismatch (expected 3)");
        if (link_qpid_tracking_[link_ee_name_l_].size() != TASK_DOF)     RCLCPP_WARN(node->get_logger(), "QPID_gains.tracking.left size mismatch (expected 6)");
        if (link_qpid_tracking_[link_ee_name_r_].size() != TASK_DOF)     RCLCPP_WARN(node->get_logger(), "QPID_gains.tracking.right size mismatch (expected 6)");
        if (qpid_mani_vel_damping_vec.size()            != MANI_DOF)     RCLCPP_WARN(node->get_logger(), "QPID_gains.mani_vel_damping size mismatch (expected 14)");
        if (qpid_mani_acc_damping_vec.size()            != MANI_DOF)     RCLCPP_WARN(node->get_logger(), "QPID_gains.mani_acc_damping size mismatch (expected 14)");
        if (qpid_base_vel_damping_vec.size()            != 3)            RCLCPP_WARN(node->get_logger(), "QPID_gains.base_vel_damping size mismatch (expected 3)");
        if (qpid_base_acc_damping_vec.size()            != 3)            RCLCPP_WARN(node->get_logger(), "QPID_gains.base_acc_damping size mismatch (expected 3)");

        robot_controller_->setManipulatorJointGain(mani_joint_kp_, mani_joint_kv_);
        robot_controller_->setTaskGain(link_task_kp_, link_task_kv_);
        robot_controller_->setQPIKGain(link_qpik_tracking_, qpik_mani_damping_, qpik_base_damping_);
        robot_controller_->setQPIDGain(link_qpid_tracking_, qpid_mani_vel_damping_, qpid_mani_acc_damping_, qpid_base_vel_damping_, qpid_base_acc_damping_);


        std::ostringstream oss;
        oss << "\n=================================================================\n"
            << "=================================================================\n"
            << "URDF Joint Information: DualFR3Husky\n"
            << robot_data_->getVerbose()
            << "=================================================================\n"
            << "=================================================================";
        const std::string print_info = oss.str();
        RCLCPP_INFO(node->get_logger(), "%s%s%s", cblue, print_info.c_str(), creset);
    }

    void DualFR3HuskyController::starting()
    {
        current_l_ee_pose_pub_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),  std::bind(&DualFR3HuskyController::pubLEEPoseCallback, this));
        current_r_ee_pose_pub_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),  std::bind(&DualFR3HuskyController::pubREEPoseCallback, this));
        current_base_pose_pub_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DualFR3HuskyController::pubBasePoseCallback, this));
        current_base_vel_pub_timer_  = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DualFR3HuskyController::pubBaseVelCallback, this));
    }

    void DualFR3HuskyController::updateState(const MujocoRosSim::VecMap& pos_dict, 
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
        for(size_t i=0; i<int(MANI_DOF/2); i++)
        {
            const std::string& l_name = "fr3_l_joint" + std::to_string(i+1);
            const std::string& r_name = "fr3_r_joint" + std::to_string(i+1);
            q_mani_(i) = pos_dict.at(l_name)(0);
            qdot_mani_(i) = vel_dict.at(l_name)(0);
            q_mani_(i + int(MANI_DOF/2)) = pos_dict.at(r_name)(0);
            qdot_mani_(i + int(MANI_DOF/2)) = vel_dict.at(r_name)(0);
        }

        // get mobile wheel joint
        q_mobile_(0) = pos_dict.at("front_left_wheel")(0);
        q_mobile_(1) = pos_dict.at("front_right_wheel")(0);
        qdot_mobile_(0) = vel_dict.at("front_left_wheel")(0);
        qdot_mobile_(1) = vel_dict.at("front_right_wheel")(0);

        if(!robot_data_->updateState(q_virtual_, q_mobile_, q_mani_, qdot_virtual_, qdot_mobile_, qdot_mani_))
        {
            RCLCPP_ERROR(node_->get_logger(), "%sFailed to update robot state.%s", cred, creset);
        }

        // get ee
        link_ee_task_[link_ee_name_l_].x    = robot_data_->getPose(link_ee_name_l_);
        link_ee_task_[link_ee_name_r_].x    = robot_data_->getPose(link_ee_name_r_);
        link_ee_task_[link_ee_name_l_].xdot = robot_data_->getVelocity(link_ee_name_l_);
        link_ee_task_[link_ee_name_r_].xdot = robot_data_->getVelocity(link_ee_name_r_);
        
    }

    void DualFR3HuskyController::updateRGBDImage(const MujocoRosSim::ImageCVMap& images)
    {

    }

    void DualFR3HuskyController::compute()
    {
        if(is_mode_changed_)
        {
            is_mode_changed_ = false;

            control_start_time_ = current_time_;

            base_vel_init_= base_vel_;
            base_vel_desired_.setZero();

            q_virtual_init_    = q_virtual_;
            qdot_virtual_init_ = qdot_virtual_;
            q_virtual_desired_ = q_virtual_init_;
            qdot_virtual_desired_.setZero();

            q_mani_init_    = q_mani_;
            qdot_mani_init_ = qdot_mani_;
            q_mani_desired_ = q_mani_init_;
            qdot_mani_desired_.setZero();
            
            q_mobile_init_    = q_mobile_;
            qdot_mobile_init_ = qdot_mobile_;
            q_mobile_desired_ = q_mobile_init_;
            qdot_mobile_desired_.setZero();

            x_l_goal_ = link_ee_task_[link_ee_name_l_].x;
            x_r_goal_ = link_ee_task_[link_ee_name_r_].x;

            link_ee_task_[link_ee_name_l_].setInit();
            link_ee_task_[link_ee_name_l_].setDesired();
            link_ee_task_[link_ee_name_l_].xdot.setZero();
            link_ee_task_[link_ee_name_r_].setInit();
            link_ee_task_[link_ee_name_r_].setDesired();
            link_ee_task_[link_ee_name_r_].xdot.setZero();
        }

        if(mode_ == "QPIK" || mode_ == "QPID")
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
            ManiVec q_mani_target;
            q_mani_target << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4, 
                             0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
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
            MobiVec qdot_mobile_desired;
            ManiVec qdot_mani_desired;
            if(robot_controller_->QPIKCubic(link_ee_task_, current_time_, control_start_time_, 4.0, qdot_mobile_desired, qdot_mani_desired))
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
        else if(mode_ == "QPID")
        {
            MobiVec qddot_mobile_desired;
            ManiVec torque_mani_desired;
            if(robot_controller_->QPIDCubic(link_ee_task_, current_time_, control_start_time_, 4.0, qddot_mobile_desired,torque_mani_desired))
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
            link_ee_task_[link_ee_name_l_].xddot_desired.setZero();
            link_ee_task_[link_ee_name_r_].xddot_desired.setZero();
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

    MujocoRosSim::CtrlInputMap DualFR3HuskyController::getCtrlInput() const
    {
        MujocoRosSim::CtrlInputMap ctrl_dict;
        ctrl_dict["left_wheel"] = qdot_mobile_desired_(0);
        ctrl_dict["right_wheel"] = qdot_mobile_desired_(1);
        for(size_t i=0; i<int(MANI_DOF/2); i++)
        {
            const std::string l_name = "fr3_l_joint" + std::to_string(i+1);
            const std::string r_name = "fr3_r_joint" + std::to_string(i+1);
            ctrl_dict[l_name] = torque_mani_desired_(i);
            ctrl_dict[r_name] = torque_mani_desired_(int(MANI_DOF/2)+i);
        }

        return ctrl_dict;
    }

    void DualFR3HuskyController::setMode(const std::string& mode)
    {
        is_mode_changed_ = true;
        mode_ = mode;
        RCLCPP_INFO(node_->get_logger(), "\033[34m Mode changed: %s\033[0m", mode.c_str());
    }

    void DualFR3HuskyController::keyCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Key input received: %d", msg->data);
        if(msg->data == 1)      setMode("HOME");
        else if(msg->data == 2) setMode("QPIK");
        else if(msg->data == 3) setMode("QPID");
        else if(msg->data == 4) setMode("Gravity_compensattion_W_QPID");
        else if(msg->data == 5) setMode("Base Velocity Tracking");
        else                    setMode("NONE");
    }
    
    void DualFR3HuskyController::subtargetLEEPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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

    void DualFR3HuskyController::subtargetREEPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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

    void DualFR3HuskyController::subtargetBaseVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Target base velocity received: linear=(%.3f, %.3f, %.3f), "
                    "angular=(%.3f, %.3f, %.3f)",
                    msg->linear.x, msg->linear.y, msg->linear.z,
                    msg->angular.x, msg->angular.y, msg->angular.z);

        base_vel_desired_.head(2) << msg->linear.x, msg->linear.y;
        base_vel_desired_(2) = msg->angular.z;
    }

    void DualFR3HuskyController::subJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
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

    void DualFR3HuskyController::pubLEEPoseCallback()
    {
        auto ee_pose_msg = geometry_msgs::msg::PoseStamped();
        ee_pose_msg.header.frame_id = "world";
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

    void DualFR3HuskyController::pubREEPoseCallback()
    {
        auto ee_pose_msg = geometry_msgs::msg::PoseStamped();
        ee_pose_msg.header.frame_id = "world";
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

    void DualFR3HuskyController::pubBasePoseCallback()
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

    void DualFR3HuskyController::pubBaseVelCallback()
    {
        auto base_vel_msg = geometry_msgs::msg::Twist();
        base_vel_msg.linear.x = base_vel_(0);
        base_vel_msg.linear.y = base_vel_(1);
        base_vel_msg.angular.z = base_vel_(2);
        
        current_base_vel_pub_->publish(base_vel_msg);
    }


    /* register with the global registry */
    PLUGINLIB_EXPORT_CLASS(DualFR3Husky::DualFR3HuskyController, MujocoRosSim::ControllerInterface)
} // namespace DualFR3Husky
