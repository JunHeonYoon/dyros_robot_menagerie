#pragma once
#include "dyros_robot_controller/mobile/robot_data.h"

namespace XLS
{
/*
URDF Joint Information: XLS
 name                | value
---------------------+---------------------------
type                 | Mecanum
wheel_num            | 4
wheel_radius         | 0.1200

roller_angles (rad)
 idx | value
-----+------------
   0 |    -0.7854
   1 |     0.7854
   2 |     0.7854
   3 |    -0.7854

base2wheel_positions
 idx | position
-----+-------------------------
   0 | [0.2225  0.2045]
   1 | [ 0.2225  -0.2045]
   2 | [-0.2225   0.2045]
   3 | [-0.2225  -0.2045]

base2wheel_angles (rad)
 idx | value
-----+------------
   0 |     0.0000
   1 |     0.0000
   2 |     0.0000
   3 |     0.0000
*/
    inline constexpr int TASK_DOF  = 3;
    inline constexpr int WHEEL_DOF = 4;

    typedef Eigen::Matrix<double,TASK_DOF,1>  TaskVec;
    typedef Eigen::Matrix<double,WHEEL_DOF,1> WheelVec;

    class XLSRobotData : public drc::Mobile::RobotData
    {
        public: 
            XLSRobotData();
    };
} // namespace XLS