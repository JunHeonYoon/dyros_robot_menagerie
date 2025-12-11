#include "dyros_robot_menagerie/dual_fr3/robot_data.h"

namespace DualFR3
{

    DualFR3RobotData::DualFR3RobotData()
    : drc::Manipulator::RobotData(
            ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/dual_fr3.urdf",
            ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/dual_fr3.srdf",
            ament_index_cpp::get_package_share_directory("mujoco_ros_sim")) 
    {
        ee_l_name_ = "fr3_l_hand_tcp";
        ee_r_name_ = "fr3_r_hand_tcp";
        ee_name_vec_.resize(2);
        ee_name_vec_[0] = ee_l_name_;
        ee_name_vec_[1] = ee_r_name_;
    }
} // namespace DualFR3