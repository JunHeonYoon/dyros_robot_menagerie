#pragma once
#include "dyros_robot_controller/manipulator/robot_data.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace DualFR3
{
/*
URDF Joint Information: DualFR3
Total nq = 14
Total nv = 14

 id | name                 | nq | nv | idx_q | idx_v
----+----------------------+----+----+-------+------
  1 |         fr3_l_joint1 |  1 |  1 |     0 |    0
  2 |         fr3_l_joint2 |  1 |  1 |     1 |    1
  3 |         fr3_l_joint3 |  1 |  1 |     2 |    2
  4 |         fr3_l_joint4 |  1 |  1 |     3 |    3
  5 |         fr3_l_joint5 |  1 |  1 |     4 |    4
  6 |         fr3_l_joint6 |  1 |  1 |     5 |    5
  7 |         fr3_l_joint7 |  1 |  1 |     6 |    6
  8 |         fr3_r_joint1 |  1 |  1 |     7 |    7
  9 |         fr3_r_joint2 |  1 |  1 |     8 |    8
 10 |         fr3_r_joint3 |  1 |  1 |     9 |    9
 11 |         fr3_r_joint4 |  1 |  1 |    10 |   10
 12 |         fr3_r_joint5 |  1 |  1 |    11 |   11
 13 |         fr3_r_joint6 |  1 |  1 |    12 |   12
 14 |         fr3_r_joint7 |  1 |  1 |    13 |   13
*/

    inline constexpr int TASK_DOF     = 6;
    inline constexpr int JOINT_DOF    = 14;

    typedef Eigen::Matrix<double,TASK_DOF,1>     TaskVec;
    typedef Eigen::Matrix<double,JOINT_DOF,1>    JointVec;

    class DualFR3RobotData : public drc::Manipulator::RobotData
    {
        public:
            DualFR3RobotData(const double dt);
    };
} // namespace DualFR3
