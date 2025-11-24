#include "dyros_robot_menagerie/fr3_pcv/robot_data.h"

namespace FR3PCV
{
    static drc::Mobile::KinematicParam makeMobileParam()
    {
        drc::Mobile::KinematicParam p;
        p.type = drc::Mobile::DriveType::Caster;
        p.wheel_radius = 0.055;
        p.wheel_offset = 0.020;
        p.base2wheel_positions = {Vector2d( 0.215,  0.125),  // front_left
                                  Vector2d( 0.215, -0.125),  // front_right
                                  Vector2d(-0.215,  0.125),  // rear_left
                                  Vector2d(-0.215, -0.125)}; // rear_right
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

    FR3PCVRobotData::FR3PCVRobotData()
    : drc::MobileManipulator::RobotData(
        makeMobileParam(),
        makeJointIndex(),
        makeActuatorIndex(),
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3_pcv.urdf",
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3_pcv.srdf")
    {
        ee_name_ = "fr3_hand_tcp";
    }

    Affine3d FR3PCVRobotData::computePose(const VectorXd& q_virtual,
                                            const VectorXd& q_mobile,
                                            const VectorXd& q_mani)
    {
        return drc::MobileManipulator::RobotData::computePose(q_virtual,q_mobile,q_mani,ee_name_);
    }

    MatrixXd FR3PCVRobotData::computeJacobian(const VectorXd& q_virtual,
                                                const VectorXd& q_mobile,
                                                const VectorXd& q_mani)
    {
        return drc::MobileManipulator::RobotData::computeJacobian(q_virtual,q_mobile,q_mani,ee_name_);
    }
    MatrixXd FR3PCVRobotData::computeJacobianTimeVariation(const VectorXd& q_virtual,
                                                             const VectorXd& q_mobile,
                                                             const VectorXd& q_mani,
                                                             const VectorXd& qdot_virtual,
                                                             const VectorXd& qdot_mobile,
                                                             const VectorXd& qdot_mani)
    {
        return drc::MobileManipulator::RobotData::computeJacobianTimeVariation(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    VectorXd FR3PCVRobotData::computeVelocity(const VectorXd& q_virtual,
                                                const VectorXd& q_mobile,
                                                const VectorXd& q_mani,
                                                const VectorXd& qdot_virtual,
                                                const VectorXd& qdot_mobile,
                                                const VectorXd& qdot_mani)
    {
        return drc::MobileManipulator::RobotData::computeVelocity(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    MatrixXd FR3PCVRobotData::computeJacobianActuated(const VectorXd& q_virtual,
                                                        const VectorXd& q_mobile,
                                                        const VectorXd& q_mani)
    {
        return drc::MobileManipulator::RobotData::computeJacobianActuated(q_virtual,q_mobile,q_mani,ee_name_);
    }

    MatrixXd FR3PCVRobotData::computeJacobianTimeVariationActuated(const VectorXd& q_virtual,
                                                                     const VectorXd& q_mobile,
                                                                     const VectorXd& q_mani,
                                                                     const VectorXd& qdot_virtual,
                                                                     const VectorXd& qdot_mobile,
                                                                     const VectorXd& qdot_mani)
    {
        return drc::MobileManipulator::RobotData::computeJacobianTimeVariationActuated(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    drc::Manipulator::ManipulabilityResult FR3PCVRobotData::computeManipulability(const VectorXd& q_mani, 
                                                                                          const VectorXd& qdot_mani, 
                                                                                          const bool& with_grad, 
                                                                                          const bool& with_graddot)
    {
        return drc::MobileManipulator::RobotData::computeManipulability(q_mani,qdot_mani,with_grad,with_graddot,ee_name_);
    }

    Affine3d FR3PCVRobotData::getPose() const
    {
        return drc::MobileManipulator::RobotData::getPose(ee_name_);
    }

    MatrixXd FR3PCVRobotData::getJacobian()
    {
        return drc::MobileManipulator::RobotData::getJacobian(ee_name_);
    }

    MatrixXd FR3PCVRobotData::getJacobianTimeVariation()
    {
        return drc::MobileManipulator::RobotData::getJacobianTimeVariation(ee_name_);
    } 

    VectorXd FR3PCVRobotData::getVelocity()
    {
        return drc::MobileManipulator::RobotData::getVelocity(ee_name_);
    }   

    drc::Manipulator::ManipulabilityResult FR3PCVRobotData::getManipulability(const bool& with_grad, const bool& with_graddot)
    {
        return drc::MobileManipulator::RobotData::getManipulability(with_grad,with_graddot,ee_name_);
    }
    
    MatrixXd FR3PCVRobotData::getJacobianActuated()
    {
        return drc::MobileManipulator::RobotData::getJacobianActuated(ee_name_);
    }

    MatrixXd FR3PCVRobotData::getJacobianActuatedTimeVariation()
    {
        return drc::MobileManipulator::RobotData::getJacobianActuatedTimeVariation(ee_name_);
    }
} // namespace FR3PCV