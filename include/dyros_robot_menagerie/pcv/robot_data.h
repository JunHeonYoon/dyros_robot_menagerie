#pragma once
#include "dyros_robot_controller/mobile/robot_data.h"

namespace PCV
{
/*
URDF Joint Information: PCV
 name                | value
---------------------+---------------------------
type                 | Caster
wheel_num            | 8
wheel_radius         | 0.0550
offset               | 0.0200

base2wheel_positions
 idx | position
-----+-------------------------
   0 | [0.2150  0.1250]
   1 | [ 0.2150  -0.1250]
   2 | [-0.2150   0.1250]
   3 | [-0.2150  -0.1250]
*/
    inline constexpr int TASK_DOF  = 3;
    inline constexpr int WHEEL_DOF = 8;

    typedef Eigen::Matrix<double,TASK_DOF,1>  TaskVec;
    typedef Eigen::Matrix<double,WHEEL_DOF,1> WheelVec;

    class PCVRobotData : public drc::Mobile::RobotData
    {
        public: 
            PCVRobotData();
    };
} // namespace PCV