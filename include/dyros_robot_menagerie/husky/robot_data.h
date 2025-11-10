#pragma once
#include "dyros_robot_controller/mobile/robot_data.h"

namespace Husky
{
/*
URDF Joint Information: Husky
 name                | value
---------------------+---------------------------
type                 | Differential
wheel_num            | 2
wheel_radius         | 0.1651
base_width           | 1.0702
*/
    inline constexpr int TASK_DOF  = 3;
    inline constexpr int WHEEL_DOF = 2;

    typedef Eigen::Matrix<double,TASK_DOF,1>  TaskVec;
    typedef Eigen::Matrix<double,WHEEL_DOF,1> WheelVec;

    class HuskyRobotData : public drc::Mobile::RobotData
    {
        public: 
            HuskyRobotData();
    };
} // namespace Husky