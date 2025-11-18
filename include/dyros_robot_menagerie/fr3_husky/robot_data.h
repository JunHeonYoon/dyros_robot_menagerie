#pragma once
#include "dyros_robot_controller/mobile_manipulator/robot_data.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace FR3Husky
{
/*
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
            FR3HuskyRobotData();
            Affine3d computePose(const VectorXd& q_virtual,
                                 const VectorXd& q_mobile,
                                 const VectorXd& q_mani);
            MatrixXd computeJacobian(const VectorXd& q_virtual,
                                     const VectorXd& q_mobile,
                                     const VectorXd& q_mani);
            MatrixXd computeJacobianTimeVariation(const VectorXd& q_virtual,
                                                  const VectorXd& q_mobile,
                                                  const VectorXd& q_mani,
                                                  const VectorXd& qdot_virtual,
                                                  const VectorXd& qdot_mobile,
                                                  const VectorXd& qdot_mani);
            VectorXd computeVelocity(const VectorXd& q_virtual,
                                     const VectorXd& q_mobile,
                                     const VectorXd& q_mani,
                                     const VectorXd& qdot_virtual,
                                     const VectorXd& qdot_mobile,
                                     const VectorXd& qdot_mani);

            MatrixXd computeJacobianActuated(const VectorXd& q_virtual,
                                             const VectorXd& q_mobile,
                                             const VectorXd& q_mani);
            MatrixXd computeJacobianTimeVariationActuated(const VectorXd& q_virtual,
                                                          const VectorXd& q_mobile,
                                                          const VectorXd& q_mani,
                                                          const VectorXd& qdot_virtual,
                                                          const VectorXd& qdot_mobile,
                                                          const VectorXd& qdot_mani);

            drc::Manipulator::ManipulabilityResult computeManipulability(const VectorXd& q_mani, 
                                                                               const VectorXd& qdot_mani, 
                                                                               const bool& with_grad, 
                                                                               const bool& with_graddot);
            Affine3d getPose() const;
            MatrixXd getJacobian();
            MatrixXd getJacobianTimeVariation(); 
            VectorXd getVelocity();
            drc::Manipulator::ManipulabilityResult getManipulability(const bool& with_grad, const bool& with_graddot);
            MatrixXd getJacobianActuated();
            MatrixXd getJacobianActuatedTimeVariation();
            std::string getEEName(){return ee_name_;}

        private:
            std::string ee_name_;
    };
} // namespace FR3Husky