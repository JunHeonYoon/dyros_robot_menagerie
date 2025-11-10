#pragma once
#include "dyros_robot_controller/manipulator/robot_data.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace UR5e
{
/*
URDF Joint Information: UR5e
Total nq = 6
Total nv = 6

 id | name                 | nq | nv | idx_q | idx_v
----+----------------------+----+----+-------+------
  1 |   shoulder_pan_joint |  1 |  1 |     0 |    0
  2 |  shoulder_lift_joint |  1 |  1 |     1 |    1
  3 |          elbow_joint |  1 |  1 |     2 |    2
  4 |        wrist_1_joint |  1 |  1 |     3 |    3
  5 |        wrist_2_joint |  1 |  1 |     4 |    4
  6 |        wrist_3_joint |  1 |  1 |     5 |    5
*/

    inline constexpr int TASK_DOF  = 6;
    inline constexpr int JOINT_DOF = 6;

    typedef Eigen::Matrix<double,TASK_DOF,1>  TaskVec;
    typedef Eigen::Matrix<double,JOINT_DOF,1> JointVec;

    class UR5eRobotData : public drc::Manipulator::RobotData
    {
        public:
            UR5eRobotData(); 

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
} // namespace UR5e