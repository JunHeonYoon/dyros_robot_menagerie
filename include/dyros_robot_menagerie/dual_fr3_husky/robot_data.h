#pragma once
#include "dyros_robot_controller/mobile_manipulator/robot_data.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace DualFR3Husky
{
/*
URDF Joint Information: DualFR3Husky
Total nq = 19
Total nv = 19

 id | name                 | nq | nv | idx_q | idx_v
----+----------------------+----+----+-------+------
  1 |            v_x_joint |  1 |  1 |     0 |    0
  2 |            v_y_joint |  1 |  1 |     1 |    1
  3 |            v_t_joint |  1 |  1 |     2 |    2
  4 |         fr3_l_joint1 |  1 |  1 |     3 |    3
  5 |         fr3_l_joint2 |  1 |  1 |     4 |    4
  6 |         fr3_l_joint3 |  1 |  1 |     5 |    5
  7 |         fr3_l_joint4 |  1 |  1 |     6 |    6
  8 |         fr3_l_joint5 |  1 |  1 |     7 |    7
  9 |         fr3_l_joint6 |  1 |  1 |     8 |    8
 10 |         fr3_l_joint7 |  1 |  1 |     9 |    9
 11 |         fr3_r_joint1 |  1 |  1 |    10 |   10
 12 |         fr3_r_joint2 |  1 |  1 |    11 |   11
 13 |         fr3_r_joint3 |  1 |  1 |    12 |   12
 14 |         fr3_r_joint4 |  1 |  1 |    13 |   13
 15 |         fr3_r_joint5 |  1 |  1 |    14 |   14
 16 |         fr3_r_joint6 |  1 |  1 |    15 |   15
 17 |         fr3_r_joint7 |  1 |  1 |    16 |   16
 18 |           left_wheel |  1 |  1 |    17 |   17
 19 |          right_wheel |  1 |  1 |    18 |   18

==================== Partition Indices ====================
 joint index
 name                | start
---------------------+------
 virtual             | 0
 manipulator         | 3
 mobile              | 17

 actuator index
 name                | start
---------------------+------
 manipulator         | 0
 mobile              | 14

======================= DoF Summary =======================
 total dof           | 19
 virtual dof         | 3
 mobile dof          | 2
 manipulator dof     | 14
 actuated dof        | 16

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
    inline constexpr int MANI_DOF     = 14;
    inline constexpr int MOBI_DOF     = 2; // differential wheel
    inline constexpr int ACTUATOR_DOF = MANI_DOF + MOBI_DOF;
    inline constexpr int JOINT_DOF    = ACTUATOR_DOF + VIRTUAL_DOF;

    typedef Eigen::Matrix<double,TASK_DOF,1>     TaskVec;
    typedef Eigen::Matrix<double,VIRTUAL_DOF,1>  VirtualVec;
    typedef Eigen::Matrix<double,MANI_DOF,1>     ManiVec;
    typedef Eigen::Matrix<double,MOBI_DOF,1>     MobiVec;
    typedef Eigen::Matrix<double,ACTUATOR_DOF,1> AactuatorVec;
    typedef Eigen::Matrix<double,JOINT_DOF,1>    JointVec;

    class DualFR3HuskyRobotData : public drc::MobileManipulator::RobotData
    {
        public:
            DualFR3HuskyRobotData();
            std::string getLEEName(){return ee_l_name_;}
            std::string getREEName(){return ee_r_name_;}
            std::vector<std::string> getEENameVec(){return ee_name_vec_;}

        private:
            std::vector<std::string> ee_name_vec_;
            std::string ee_l_name_;
            std::string ee_r_name_;
    };
} // namespace DualFR3Husky