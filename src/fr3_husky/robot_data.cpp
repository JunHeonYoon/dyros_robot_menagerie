#include "dyros_robot_menagerie/fr3_husky/robot_data.h"

namespace FR3Husky
{
    static drc::Mobile::KinematicParam makeMobileParam()
    {
        drc::Mobile::KinematicParam p;
        p.type         = drc::Mobile::DriveType::Differential;
        p.wheel_radius = 0.1651;
        p.base_width   = 0.2854 * 2 * 1.875;
        p.max_lin_speed  = 1;
        p.max_ang_speed  = 1;
        p.max_lin_acc  = 1;
        p.max_ang_acc  = 1;
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

    FR3HuskyRobotData::FR3HuskyRobotData(const double dt)
    : drc::MobileManipulator::RobotData(
        dt,
        makeMobileParam(),
        makeJointIndex(),
        makeActuatorIndex(),
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3_husky.urdf",
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3_husky.srdf")
    {
    }
} // namespace FR3Husky
