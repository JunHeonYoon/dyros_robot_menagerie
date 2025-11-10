#pragma once
#include "dyros_robot_controller/manipulator/robot_data.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace FR3
{
/*
URDF Joint Information: FR3
Total nq = 7
Total nv = 7

 id | name                 | nq | nv | idx_q | idx_v
----+----------------------+----+----+-------+------
  1 |           fr3_joint1 |  1 |  1 |     0 |    0
  2 |           fr3_joint2 |  1 |  1 |     1 |    1
  3 |           fr3_joint3 |  1 |  1 |     2 |    2
  4 |           fr3_joint4 |  1 |  1 |     3 |    3
  5 |           fr3_joint5 |  1 |  1 |     4 |    4
  6 |           fr3_joint6 |  1 |  1 |     5 |    5
  7 |           fr3_joint7 |  1 |  1 |     6 |    6
*/

    inline constexpr int TASK_DOF  = 6;
    inline constexpr int JOINT_DOF = 7;

    typedef Eigen::Matrix<double,TASK_DOF,1>  TaskVec;
    typedef Eigen::Matrix<double,JOINT_DOF,1> JointVec;

    class FR3RobotData : public drc::Manipulator::RobotData
    {
        public:
            FR3RobotData(); 

            Affine3d computePose(const VectorXd& q);
            MatrixXd computeJacobian(const VectorXd& q);
            MatrixXd computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot);
            VectorXd computeVelocity(const VectorXd& q, const VectorXd& qdot);
            drc::Manipulator::ManipulabilityResult computeManipulability(const VectorXd& q, const VectorXd& qdot, const bool& with_grad, const bool& with_graddot);
            
            Affine3d getPose() const;
            MatrixXd getJacobian();
            MatrixXd getJacobianTimeVariation(); 
            VectorXd getVelocity();
            drc::Manipulator::ManipulabilityResult getManipulability(const bool& with_grad, const bool& with_graddot);
            std::string getEEName(){return ee_name_;}
        
        private:
            std::string ee_name_;
    };
} // namespace FR3