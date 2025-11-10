#include "dyros_robot_menagerie/xls/robot_data.h"

namespace XLS
{
    static drc::Mobile::KinematicParam makeParam()
    {
        drc::Mobile::KinematicParam p;
        p.type = drc::Mobile::DriveType::Mecanum;
        p.wheel_radius = 0.120;
        p.base2wheel_positions = {Vector2d( 0.2225,  0.2045),  // front_left
                                  Vector2d( 0.2225, -0.2045),  // front_right
                                  Vector2d(-0.2225,  0.2045),  // rear_left
                                  Vector2d(-0.2225, -0.2045)}; // rear_right
        p.base2wheel_angles = {0, 0, 0, 0};
        p.roller_angles = {-M_PI/4,  // front_left
                            M_PI/4,  // front_right
                            M_PI/4,  // rear_left
                           -M_PI/4}; // rear_right
        p.max_lin_acc  = 3;
        p.max_ang_acc  = 6;
        return p;
    }

    XLSRobotData::XLSRobotData()
    : drc::Mobile::RobotData(makeParam())
    {
        
    }
} // namespace XLS