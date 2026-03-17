#include "dyros_robot_menagerie/fr3/controller.h"

namespace FR3
{
    void FR3Controller::configure(const rclcpp::Node::SharedPtr& node)
    {
        MujocoRosSim::ControllerInterface::configure(node);
        
        dt_ = 0.001;
        robot_data_ = std::make_shared<FR3RobotData>(dt_);
        robot_controller_ = std::make_unique<drc::Manipulator::RobotController>(robot_data_);

        rclcpp::QoS qos(rclcpp::KeepLast(1)); 
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort); 
        qos.durability(rclcpp::DurabilityPolicy::Volatile);
        
        key_sub_ = node_->create_subscription<std_msgs::msg::Int32>("fr3_controller/mode_input", 10,std::bind(&FR3Controller::keyCallback, this, std::placeholders::_1));
        target_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("fr3_controller/target_pose", 10,std::bind(&FR3Controller::subtargetPoseCallback, this, std::placeholders::_1));
        
        ee_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("fr3_controller/ee_pose", 10);
        hand_eye_rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("fr3_controller/handeye/rgb/image_raw", qos);
        hand_eye_depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("fr3_controller/handeye/depth/image_raw", qos);
        
        q_.setZero();
        q_desired_.setZero();
        q_init_.setZero();
        qdot_.setZero();
        qdot_desired_.setZero();
        qdot_init_.setZero();
        
        x_goal_.setIdentity();
        ee_name_ = "fr3_hand_tcp";
        link_ee_task_[ee_name_] = drc::TaskSpaceData::Zero();
        
        torque_desired_.setZero();

        setDRCGains();

        std::ostringstream oss;
        oss << "\n=================================================================\n"
            << "=================================================================\n"
            << "URDF Joint Information: FR3\n"
            << robot_data_->getVerbose()
            << "=================================================================\n"
            << "=================================================================";
        const std::string print_info = oss.str();
        RCLCPP_INFO(node->get_logger(), "%s%s%s", cblue, print_info.c_str(), creset);
    }
    
    void FR3Controller::starting()
    {
        ee_pose_pub_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), std::bind(&FR3Controller::pubEEPoseCallback, this));
        hand_eye_cam_pub_timer_ = node_->create_wall_timer(std::chrono::milliseconds(16), std::bind(&FR3Controller::pubHandEyeCallback, this));
    }

    void FR3Controller::updateState(const MujocoRosSim::VecMap& pos_dict, 
                                    const MujocoRosSim::VecMap& vel_dict,
                                    const MujocoRosSim::VecMap& tau_ext_dict, 
                                    const MujocoRosSim::VecMap& sensors_dict, 
                                    double current_time)
    {
        current_time_ = current_time;

        // get manipulator joint
        for(size_t i=0; i<JOINT_DOF; i++)
        {
            const std::string& name = "fr3_joint" + std::to_string(i+1);
            q_(i) = pos_dict.at(name)(0);
            qdot_(i) = vel_dict.at(name)(0);
        }

        if(!robot_data_->updateState(q_, qdot_)) RCLCPP_ERROR(node_->get_logger(), "%sFailed to update robot state.%s", cred, creset);

        // get ee
        link_ee_task_[ee_name_].x    = robot_data_->getPose(ee_name_);
        link_ee_task_[ee_name_].xdot = robot_data_->getVelocity(ee_name_);
    }

    void FR3Controller::updateRGBDImage(const MujocoRosSim::ImageCVMap& images)
    {
        std::scoped_lock<std::mutex> lk(hand_eye_cam_mtx_);
        hand_eye_rgb_img_ = images.at("hand_eye").rgb.clone();
        hand_eye_depth_img_ = images.at("hand_eye").depth.clone();
    }

    void FR3Controller::compute()
    {
        if(is_mode_changed_)
        {
            is_mode_changed_ = false;
            control_start_time_ = current_time_;
            
            q_init_ = q_;
            qdot_init_ = qdot_;
            q_desired_ = q_init_;
            qdot_desired_.setZero();
            
            x_goal_ = link_ee_task_[ee_name_].x;
            
            link_ee_task_[ee_name_].setInit();
            link_ee_task_[ee_name_].setDesired();
            link_ee_task_[ee_name_].xdot.setZero();
            link_ee_task_[ee_name_].control_start_time = current_time_;
        }

        if(mode_ == "CLIK" || mode_ == "OSF" || mode_ == "QPIK" || mode_ == "QPID")
        {
            if(is_goal_pose_changed_)
            {
                link_ee_task_[ee_name_].control_start_time = current_time_;
                control_start_time_ = current_time_;
                
                link_ee_task_[ee_name_].setInit();
                link_ee_task_[ee_name_].xdot.setZero();
                link_ee_task_[ee_name_].x_desired = x_goal_;

                is_goal_pose_changed_ = false;
            }
        }

        if(mode_ == "HOME")
        {
            JointVec target_q;
            target_q << 0.0, 0.0, 0.0, -M_PI/2., 0.0, M_PI/2., M_PI / 4.;
            torque_desired_ = robot_controller_->moveJointTorqueCubic(target_q,
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
            if (!robot_controller_->CLIKCubic(link_ee_task_, current_time_, 4.0, qdot_desired_)) qdot_desired_.setZero();
            q_desired_ += dt_ * qdot_desired_;
            torque_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
        }
        else if(mode_ == "QPIK")
        {
            if (!robot_controller_->QPIKCubic(link_ee_task_, current_time_, 4.0, qdot_desired_)) qdot_desired_.setZero();
            q_desired_ += dt_ * qdot_desired_;
            torque_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
        }
        else if(mode_ == "OSF")
        {
            JointVec target_q;
            target_q << 0.0, 0.0, 0.0, -M_PI/2., 0.0, M_PI/2., M_PI / 4.;
            VectorXd null_torque_desired = robot_controller_->moveJointTorqueStep(q_init_, JointVec::Zero(), false);
            if (!robot_controller_->OSFCubic(link_ee_task_, current_time_, 4.0, torque_desired_, null_torque_desired)) torque_desired_ = robot_data_->getGravity();
        }
        else if(mode_ == "QPID")
        {
            if (!robot_controller_->QPIDCubic(link_ee_task_, current_time_, 4.0, torque_desired_)) torque_desired_ = robot_data_->getGravity();
        }
        else if(mode_ == "Gravity_compensattion_W_QPID")
        {
            link_ee_task_[ee_name_].xddot_desired.setZero();
            if (!robot_controller_->QPID(link_ee_task_, torque_desired_)) torque_desired_ = robot_data_->getGravity();
        }
        else
        {
            torque_desired_ = robot_data_->getGravity();
        }
    }

    MujocoRosSim::CtrlInputMap FR3Controller::getCtrlInput() const
    {
        MujocoRosSim::CtrlInputMap ctrl_dict;
        for(size_t i=0; i<JOINT_DOF; i++)
        {
            const std::string name = "fr3_joint" + std::to_string(i+1);
            ctrl_dict[name] = torque_desired_(i);
        }
        return ctrl_dict;
    }

    void FR3Controller::setMode(const std::string& mode)
    {
        is_mode_changed_ = true;
        mode_ = mode;
        RCLCPP_INFO(node_->get_logger(), "%sMode changed: %s%s", cblue, mode.c_str(), creset);
    }

    void FR3Controller::keyCallback(const std_msgs::msg::Int32::SharedPtr msg)
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

    void FR3Controller::subtargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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

    void FR3Controller::pubEEPoseCallback()
    {
        auto ee_pose_msg = geometry_msgs::msg::PoseStamped();
        ee_pose_msg.header.frame_id = "base_link";
        ee_pose_msg.header.stamp = node_->now();

        ee_pose_msg.pose.position.x = link_ee_task_[ee_name_].x.translation()(0);
        ee_pose_msg.pose.position.y = link_ee_task_[ee_name_].x.translation()(1);
        ee_pose_msg.pose.position.z = link_ee_task_[ee_name_].x.translation()(2);

        Eigen::Quaterniond q(link_ee_task_[ee_name_].x.rotation());
        ee_pose_msg.pose.orientation.x = q.x();
        ee_pose_msg.pose.orientation.y = q.y();
        ee_pose_msg.pose.orientation.z = q.z();
        ee_pose_msg.pose.orientation.w = q.w();
        
        ee_pose_pub_->publish(ee_pose_msg);
    }

    void FR3Controller::pubHandEyeCallback()
    {
        cv::Mat img_rgb, img_depth;
        {
            std::scoped_lock<std::mutex> lk(hand_eye_cam_mtx_);
            if (hand_eye_rgb_img_.empty() || hand_eye_depth_img_.empty()) return;
            img_rgb = hand_eye_rgb_img_.clone();
            img_depth = hand_eye_depth_img_.clone();
        }

        auto rgb_msg = toImageMsg(img_rgb, "rgb8");
        auto depth_msg = toImageMsg(img_depth, "32FC1");
        rgb_msg->header.stamp = node_->now();
        rgb_msg->header.frame_id = "hand_eye_cam_frame";
        depth_msg->header.stamp = node_->now();
        depth_msg->header.frame_id = "hand_eye_cam_frame";
        hand_eye_rgb_pub_->publish(*rgb_msg);
        hand_eye_depth_pub_->publish(*depth_msg);
    }

    bool FR3Controller::setDRCGains()
    {
        const std::string joint = "dyros_robot_controller.manipulator_joint_gains.";
        const std::string task  = "dyros_robot_controller.task_gains.";
        const std::string qpik  = "dyros_robot_controller.QPIK_weight.";
        const std::string qpid  = "dyros_robot_controller.QPID_weight.";

        std::vector<double> joint_kp_vec = node_->declare_parameter<std::vector<double>>(joint + "kp", std::vector<double>(JOINT_DOF, 1.0));
        std::vector<double> joint_kv_vec = node_->declare_parameter<std::vector<double>>(joint + "kv", std::vector<double>(JOINT_DOF, 1.0));

        std::vector<double> task_ik_kp_vec = node_->declare_parameter<std::vector<double>>(task + "ik.kp", std::vector<double>(TASK_DOF, 1.0));
        std::vector<double> task_id_kp_vec = node_->declare_parameter<std::vector<double>>(task + "id.kp", std::vector<double>(TASK_DOF, 1.0));
        std::vector<double> task_id_kv_vec = node_->declare_parameter<std::vector<double>>(task + "id.kd", std::vector<double>(TASK_DOF, 1.0));

        std::vector<double> qpik_tracking_vec    = node_->declare_parameter<std::vector<double>>(qpik + "tracking.weights",               std::vector<double>(TASK_DOF, 1.0));
        std::vector<double> qpik_vel_damping_vec = node_->declare_parameter<std::vector<double>>(qpik + "joint.velocity.manipulator",     std::vector<double>(JOINT_DOF, 1.0));
        std::vector<double> qpik_acc_damping_vec = node_->declare_parameter<std::vector<double>>(qpik + "joint.acceleration.manipulator", std::vector<double>(JOINT_DOF, 1.0));

        std::vector<double> qpid_tracking_vec    = node_->declare_parameter<std::vector<double>>(qpid + "tracking.weights",               std::vector<double>(TASK_DOF, 1.0));
        std::vector<double> qpid_vel_damping_vec = node_->declare_parameter<std::vector<double>>(qpid + "joint.velocity.manipulator",     std::vector<double>(JOINT_DOF, 1.0));
        std::vector<double> qpid_acc_damping_vec = node_->declare_parameter<std::vector<double>>(qpid + "joint.acceleration.manipulator", std::vector<double>(JOINT_DOF, 1.0));

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
        if (!check_vector_size(joint + "kp", JOINT_DOF, joint_kp_vec.size())) return false;
        if (!check_vector_size(joint + "kv", JOINT_DOF, joint_kv_vec.size())) return false;

        // task-space gains
        if (!check_vector_size(task + "ik.kp", TASK_DOF, task_ik_kp_vec.size())) return false;
        if (!check_vector_size(task + "id.kp", TASK_DOF, task_id_kp_vec.size())) return false;
        if (!check_vector_size(task + "id.kd", TASK_DOF, task_id_kv_vec.size())) return false;

        // QPIK weights
        if (!check_vector_size(qpik + "tracking.weights",               TASK_DOF,  qpik_tracking_vec.size()))    return false;
        if (!check_vector_size(qpik + "joint.velocity.manipulator",     JOINT_DOF, qpik_vel_damping_vec.size())) return false;
        if (!check_vector_size(qpik + "joint.acceleration.manipulator", JOINT_DOF, qpik_acc_damping_vec.size())) return false;

        // QPID weights
        if (!check_vector_size(qpid + "tracking.weights",               TASK_DOF,  qpid_tracking_vec.size()))    return false;
        if (!check_vector_size(qpid + "joint.velocity.manipulator",     JOINT_DOF, qpid_vel_damping_vec.size())) return false;
        if (!check_vector_size(qpid + "joint.acceleration.manipulator", JOINT_DOF, qpid_acc_damping_vec.size())) return false;

        joint_kp_ = Eigen::Map<const Eigen::VectorXd>(joint_kp_vec.data(), joint_kp_vec.size());
        joint_kv_ = Eigen::Map<const Eigen::VectorXd>(joint_kv_vec.data(), joint_kv_vec.size());

        task_ik_kp_ = Eigen::Map<const Eigen::VectorXd>(task_ik_kp_vec.data(), task_ik_kp_vec.size());
        task_id_kp_ = Eigen::Map<const Eigen::VectorXd>(task_id_kp_vec.data(), task_id_kp_vec.size());
        task_id_kv_ = Eigen::Map<const Eigen::VectorXd>(task_id_kv_vec.data(), task_id_kv_vec.size());

        qpik_tracking_    = Eigen::Map<const Eigen::VectorXd>(qpik_tracking_vec.data(),    qpik_tracking_vec.size());
        qpik_vel_damping_ = Eigen::Map<const Eigen::VectorXd>(qpik_vel_damping_vec.data(), qpik_vel_damping_vec.size());
        qpik_acc_damping_ = Eigen::Map<const Eigen::VectorXd>(qpik_acc_damping_vec.data(), qpik_acc_damping_vec.size());
        
        qpid_tracking_    = Eigen::Map<const Eigen::VectorXd>(qpid_tracking_vec.data(),    qpid_tracking_vec.size());
        qpid_vel_damping_ = Eigen::Map<const Eigen::VectorXd>(qpid_vel_damping_vec.data(), qpid_vel_damping_vec.size());
        qpid_acc_damping_ = Eigen::Map<const Eigen::VectorXd>(qpid_acc_damping_vec.data(), qpid_acc_damping_vec.size());

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
            << "\t\tjoint.velocity.manipulator: " << qpik_vel_damping_.transpose() << "\n"
            << "\t\tjoint.acceleration.manipulator: " << qpik_acc_damping_.transpose() << "\n"
            << "\tQPID_weight" << "\n"
            << "\t\ttracking.weights: " << qpid_tracking_.transpose() << "\n"
            << "\t\tjoint.velocity.manipulator: " << qpid_vel_damping_.transpose() << "\n"
            << "\t\tjoint.acceleration.manipulator: " << qpid_acc_damping_.transpose();
        RCLCPP_INFO(node_->get_logger(), "%s%s%s", cblue, oss.str().c_str(), creset);

        robot_controller_->setJointGain(joint_kp_, joint_kv_);
        robot_controller_->setIKGain(task_ik_kp_);
        robot_controller_->setIDGain(task_id_kp_, task_id_kv_);
        robot_controller_->setQPIKGain(qpik_tracking_, qpik_vel_damping_, qpik_acc_damping_);
        robot_controller_->setQPIDGain(qpid_tracking_, qpid_vel_damping_, qpid_acc_damping_);
        return true;
    }

    /* register with the global registry */
    PLUGINLIB_EXPORT_CLASS(FR3::FR3Controller, MujocoRosSim::ControllerInterface)
} // namespace FR3
