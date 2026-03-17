#pragma once
#include "dyros_robot_controller/mobile_manipulator/robot_data.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace FR3Husky
{
/*
URDF Joint Information: FR3Husky
Total nq = 12
Total nv = 12

 id | name                 | nq | nv | idx_q | idx_v
----+----------------------+----+----+-------+------
  1 |            v_x_joint |  1 |  1 |     0 |    0
  2 |            v_y_joint |  1 |  1 |     1 |    1
  3 |            v_t_joint |  1 |  1 |     2 |    2
  4 |           fr3_joint1 |  1 |  1 |     3 |    3
  5 |           fr3_joint2 |  1 |  1 |     4 |    4
  6 |           fr3_joint3 |  1 |  1 |     5 |    5
  7 |           fr3_joint4 |  1 |  1 |     6 |    6
  8 |           fr3_joint5 |  1 |  1 |     7 |    7
  9 |           fr3_joint6 |  1 |  1 |     8 |    8
 10 |           fr3_joint7 |  1 |  1 |     9 |    9
 11 |           left_wheel |  1 |  1 |    10 |   10
 12 |          right_wheel |  1 |  1 |    11 |   11

==================== Partition Indices ====================
 joint index
 name                | start
---------------------+------
 virtual             | 0
 manipulator         | 3
 mobile              | 10

 actuator index
 name                | start
---------------------+------
 manipulator         | 0
 mobile              | 7

======================= DoF Summary =======================
 total dof           | 12
 virtual dof         | 3
 mobile dof          | 2
 manipulator dof     | 7
 actuated dof        | 9

======================= Mobile Summary =======================
 name                | value
---------------------+---------------------------
type                 | Differential
wheel_num            | 2
wheel_radius         | 0.1651
base_width           | 1.0702
*/

    inline constexpr int TASK_DOF     = 6;
    inline constexpr int VIRTUAL_DOF  = 3; // x, y, th
    inline constexpr int MANI_DOF     = 7;
    inline constexpr int MOBI_DOF     = 2; // differential wheel
    inline constexpr int ACTUATOR_DOF = MANI_DOF + MOBI_DOF;
    inline constexpr int JOINT_DOF    = ACTUATOR_DOF + VIRTUAL_DOF;

    typedef Eigen::Matrix<double,TASK_DOF,1>     TaskVec;
    typedef Eigen::Matrix<double,VIRTUAL_DOF,1>  VirtualVec;
    typedef Eigen::Matrix<double,MANI_DOF,1>     ManiVec;
    typedef Eigen::Matrix<double,MOBI_DOF,1>     MobiVec;
    typedef Eigen::Matrix<double,ACTUATOR_DOF,1> AactuatorVec;
    typedef Eigen::Matrix<double,JOINT_DOF,1>    JointVec;

    class FR3HuskyRobotData : public drc::MobileManipulator::RobotData
    {
        public:
            FR3HuskyRobotData(const double dt);
    };
} // namespace FR3Husky
