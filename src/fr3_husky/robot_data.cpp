#include "dyros_robot_menagerie/fr3_husky/robot_data.h"

namespace FR3Husky
{
    static drc::Mobile::KinematicParam makeMobileParam()
    {
        drc::Mobile::KinematicParam p;
        p.type         = drc::Mobile::DriveType::Differential;
        p.wheel_radius = 0.1651;
        p.base_width   = 0.2854 * 2 * 1.875;
        p.max_lin_acc  = 3;
        p.max_ang_acc  = 6;
        return p;
    }

    static drc::MobileManipulator::JointIndex makeJointIndex()
    {
        drc::MobileManipulator::JointIndex j;
        j.virtual_start = 0;
        j.mani_start = VIRTUAL_DOF;
        j.mobi_start = VIRTUAL_DOF + MANI_DOF;
        return j;
    }

    static drc::MobileManipulator::ActuatorIndex makeActuatorIndex()
    {
        drc::MobileManipulator::ActuatorIndex a;
        a.mani_start = 0;
        a.mobi_start = MANI_DOF;
        return a;
    }

    FR3HuskyRobotData::FR3HuskyRobotData()
    : drc::MobileManipulator::RobotData(
        makeMobileParam(),
        makeJointIndex(),
        makeActuatorIndex(),
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3_husky.urdf",
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3_husky.srdf")
    {
        ee_name_ = "fr3_hand_tcp";
    }

    Affine3d FR3HuskyRobotData::computePose(const VectorXd& q_virtual,
                                            const VectorXd& q_mobile,
                                            const VectorXd& q_mani)
    {
        return drc::MobileManipulator::RobotData::computePose(q_virtual,q_mobile,q_mani,ee_name_);
    }

    MatrixXd FR3HuskyRobotData::computeJacobian(const VectorXd& q_virtual,
                                                const VectorXd& q_mobile,
                                                const VectorXd& q_mani)
    {
        return drc::MobileManipulator::RobotData::computeJacobian(q_virtual,q_mobile,q_mani,ee_name_);
    }
    MatrixXd FR3HuskyRobotData::computeJacobianTimeVariation(const VectorXd& q_virtual,
                                                             const VectorXd& q_mobile,
                                                             const VectorXd& q_mani,
                                                             const VectorXd& qdot_virtual,
                                                             const VectorXd& qdot_mobile,
                                                             const VectorXd& qdot_mani)
    {
        return drc::MobileManipulator::RobotData::computeJacobianTimeVariation(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    VectorXd FR3HuskyRobotData::computeVelocity(const VectorXd& q_virtual,
                                                const VectorXd& q_mobile,
                                                const VectorXd& q_mani,
                                                const VectorXd& qdot_virtual,
                                                const VectorXd& qdot_mobile,
                                                const VectorXd& qdot_mani)
    {
        return drc::MobileManipulator::RobotData::computeVelocity(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    MatrixXd FR3HuskyRobotData::computeJacobianActuated(const VectorXd& q_virtual,
                                                        const VectorXd& q_mobile,
                                                        const VectorXd& q_mani)
    {
        return drc::MobileManipulator::RobotData::computeJacobianActuated(q_virtual,q_mobile,q_mani,ee_name_);
    }

    MatrixXd FR3HuskyRobotData::computeJacobianTimeVariationActuated(const VectorXd& q_virtual,
                                                                     const VectorXd& q_mobile,
                                                                     const VectorXd& q_mani,
                                                                     const VectorXd& qdot_virtual,
                                                                     const VectorXd& qdot_mobile,
                                                                     const VectorXd& qdot_mani)
    {
        return drc::MobileManipulator::RobotData::computeJacobianTimeVariationActuated(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    drc::Manipulator::ManipulabilityResult FR3HuskyRobotData::computeManipulability(const VectorXd& q_mani, 
                                                                                          const VectorXd& qdot_mani, 
                                                                                          const bool& with_grad, 
                                                                                          const bool& with_graddot)
    {
        return drc::MobileManipulator::RobotData::computeManipulability(q_mani,qdot_mani,with_grad,with_graddot,ee_name_);
    }

    Affine3d FR3HuskyRobotData::getPose() const
    {
        return drc::MobileManipulator::RobotData::getPose(ee_name_);
    }

    MatrixXd FR3HuskyRobotData::getJacobian()
    {
        return drc::MobileManipulator::RobotData::getJacobian(ee_name_);
    }

    MatrixXd FR3HuskyRobotData::getJacobianTimeVariation()
    {
        return drc::MobileManipulator::RobotData::getJacobianTimeVariation(ee_name_);
    } 

    VectorXd FR3HuskyRobotData::getVelocity()
    {
        return drc::MobileManipulator::RobotData::getVelocity(ee_name_);
    }   

    drc::Manipulator::ManipulabilityResult FR3HuskyRobotData::getManipulability(const bool& with_grad, const bool& with_graddot)
    {
        return drc::MobileManipulator::RobotData::getManipulability(with_grad,with_graddot,ee_name_);
    }
    
    MatrixXd FR3HuskyRobotData::getJacobianActuated()
    {
        return drc::MobileManipulator::RobotData::getJacobianActuated(ee_name_);
    }

    MatrixXd FR3HuskyRobotData::getJacobianActuatedTimeVariation()
    {
        return drc::MobileManipulator::RobotData::getJacobianActuatedTimeVariation(ee_name_);
    }
} // namespace FR3Husky