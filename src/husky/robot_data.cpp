#include "dyros_robot_menagerie/husky/robot_data.h"

namespace Husky
{
    static drc::Mobile::KinematicParam makeParam()
    {
        drc::Mobile::KinematicParam p;
        p.type         = drc::Mobile::DriveType::Differential;
        p.wheel_radius = 0.1651;
        p.base_width   = 0.2854 * 2 * 1.875;
        p.max_lin_acc  = 3;
        p.max_ang_acc  = 6;
        return p;
    }

    HuskyRobotData::HuskyRobotData()
    : drc::Mobile::RobotData(makeParam())
    {

    }
} // namespace Husky