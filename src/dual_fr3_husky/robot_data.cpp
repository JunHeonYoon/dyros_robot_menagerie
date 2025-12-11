#include "dyros_robot_menagerie/dual_fr3_husky/robot_data.h"

namespace DualFR3Husky
{
    static drc::Mobile::KinematicParam makeMobileParam()
    {
        drc::Mobile::KinematicParam p;
        p.type         = drc::Mobile::DriveType::Differential;
        p.wheel_radius = 0.1651;
        p.base_width   = 0.2854 * 2 * 1.875;
        p.max_lin_acc  = 3;
        p.max_ang_acc  = 6;
        return p;
    }

    static drc::MobileManipulator::JointIndex makeJointIndex()
    {
        drc::MobileManipulator::JointIndex j;
        j.virtual_start = 0;
        j.mani_start = VIRTUAL_DOF;
        j.mobi_start = VIRTUAL_DOF + MANI_DOF;
        return j;
    }

    static drc::MobileManipulator::ActuatorIndex makeActuatorIndex()
    {
        drc::MobileManipulator::ActuatorIndex a;
        a.mani_start = 0;
        a.mobi_start = MANI_DOF;
        return a;
    }

    DualFR3HuskyRobotData::DualFR3HuskyRobotData()
    : drc::MobileManipulator::RobotData(
        makeMobileParam(),
        makeJointIndex(),
        makeActuatorIndex(),
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/dual_fr3_husky.urdf",
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/dual_fr3_husky.srdf")
    {
        ee_l_name_ = "fr3_l_hand_tcp";
        ee_r_name_ = "fr3_r_hand_tcp";
        ee_name_vec_.resize(2);
        ee_name_vec_[0] = ee_l_name_;
        ee_name_vec_[1] = ee_r_name_;
    }
} // namespace DualFR3Husky