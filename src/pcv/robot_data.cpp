#include "dyros_robot_menagerie/pcv/robot_data.h"

namespace PCV
{
    static drc::Mobile::KinematicParam makeParam()
    {
        drc::Mobile::KinematicParam p;
        p.type = drc::Mobile::DriveType::Caster;
        p.wheel_radius = 0.055;
        p.wheel_offset = 0.020;
        p.base2wheel_positions = {Vector2d( 0.215,  0.125),  // front_left
                                  Vector2d( 0.215, -0.125),  // front_right
                                  Vector2d(-0.215,  0.125),  // rear_left
                                  Vector2d(-0.215, -0.125)}; // rear_right
        p.max_lin_acc  = 3;
        p.max_ang_acc  = 6;
        return p;
    }

    PCVRobotData::PCVRobotData()
    : drc::Mobile::RobotData(makeParam())
    {
        
    }
} // namespace PCV