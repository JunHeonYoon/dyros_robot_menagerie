#include "dyros_robot_menagerie/fr3/controller.h"

namespace FR3
{
    void FR3Controller::configure(const rclcpp::Node::SharedPtr& node)
    {
        MujocoRosSim::ControllerInterface::configure(node);
        dt_ = 0.001;

        robot_data_ = std::make_shared<FR3RobotData>();
        robot_controller_ = std::make_unique<drc::Manipulator::RobotController>(dt_, robot_data_);

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
        
        x_.setIdentity();
        x_init_.setIdentity();
        x_desired_.setIdentity();
        xdot_.setZero();
        xdot_init_.setZero();
        xdot_desired_.setZero();
        
        x_goal_.setIdentity();
        
        torque_desired_.setZero();
        
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
        x_ = robot_data_->getPose();
        xdot_ = robot_data_->getVelocity();
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

            x_init_ = x_;
            xdot_init_ = xdot_;
            x_desired_ = x_init_;
            xdot_desired_.setZero();

            x_goal_ = x_init_;
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
            torque_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_);
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
            torque_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_);
        }
        else if(mode_ == "OSF")
        {
            if(is_goal_pose_changed_)
            {
                control_start_time_ = current_time_;
                x_init_ = x_;
                xdot_init_ = xdot_;
                is_goal_pose_changed_ = false;
            }
            JointVec target_q;
            target_q << 0.0, 0.0, 0.0, -M_PI/2., 0.0, M_PI/2., M_PI / 4.;
            VectorXd null_torque_desired = robot_controller_->moveJointTorqueStep(q_init_, JointVec::Zero());
            torque_desired_ = robot_controller_->OSFCubic(x_goal_,
                                                          JointVec::Zero(),
                                                          x_init_,
                                                          xdot_init_,
                                                          current_time_,
                                                          control_start_time_,
                                                          4.0,
                                                          null_torque_desired,
                                                          robot_data_->getEEName());
        }
        else if(mode_ == "QPID")
        {
            if(is_goal_pose_changed_)
            {
                control_start_time_ = current_time_;
                x_init_ = x_;
                xdot_init_ = xdot_;
                is_goal_pose_changed_ = false;
            }
            torque_desired_ = robot_controller_->QPIDCubic(x_goal_,
                                                           JointVec::Zero(),
                                                           x_init_,
                                                           xdot_init_,
                                                           current_time_,
                                                           control_start_time_,
                                                           4.0,
                                                           robot_data_->getEEName());
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

    /* register with the global registry */
    PLUGINLIB_EXPORT_CLASS(FR3::FR3Controller, MujocoRosSim::ControllerInterface)
} // namespace FR3