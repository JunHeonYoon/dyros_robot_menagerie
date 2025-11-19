#include "dyros_robot_menagerie/fr3_xls/robot_data.h"

namespace FR3XLS
{
    static drc::Mobile::KinematicParam makeMobileParam()
    {
        drc::Mobile::KinematicParam p;
        p.type = drc::Mobile::DriveType::Mecanum;
        p.wheel_radius = 0.120;
        p.base2wheel_positions = {Vector2d( 0.2225,  0.2045),  // front_left
                                  Vector2d( 0.2225, -0.2045),  // front_right
                                  Vector2d(-0.2225,  0.2045),  // rear_left
                                  Vector2d(-0.2225, -0.2045)}; // rear_right
        p.base2wheel_angles = {0, 0, 0, 0};
        p.roller_angles = {-M_PI/4,  // front_left
                            M_PI/4,  // front_right
                            M_PI/4,  // rear_left
                           -M_PI/4}; // rear_right
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

    FR3XLSRobotData::FR3XLSRobotData()
    : drc::MobileManipulator::RobotData(
        makeMobileParam(),
        makeJointIndex(),
        makeActuatorIndex(),
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3_xls.urdf",
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3_xls.srdf")
    {
        ee_name_ = "fr3_hand_tcp";
    }

    Affine3d FR3XLSRobotData::computePose(const VectorXd& q_virtual,
                                            const VectorXd& q_mobile,
                                            const VectorXd& q_mani)
    {
        return drc::MobileManipulator::RobotData::computePose(q_virtual,q_mobile,q_mani,ee_name_);
    }

    MatrixXd FR3XLSRobotData::computeJacobian(const VectorXd& q_virtual,
                                                const VectorXd& q_mobile,
                                                const VectorXd& q_mani)
    {
        return drc::MobileManipulator::RobotData::computeJacobian(q_virtual,q_mobile,q_mani,ee_name_);
    }
    MatrixXd FR3XLSRobotData::computeJacobianTimeVariation(const VectorXd& q_virtual,
                                                             const VectorXd& q_mobile,
                                                             const VectorXd& q_mani,
                                                             const VectorXd& qdot_virtual,
                                                             const VectorXd& qdot_mobile,
                                                             const VectorXd& qdot_mani)
    {
        return drc::MobileManipulator::RobotData::computeJacobianTimeVariation(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    VectorXd FR3XLSRobotData::computeVelocity(const VectorXd& q_virtual,
                                                const VectorXd& q_mobile,
                                                const VectorXd& q_mani,
                                                const VectorXd& qdot_virtual,
                                                const VectorXd& qdot_mobile,
                                                const VectorXd& qdot_mani)
    {
        return drc::MobileManipulator::RobotData::computeVelocity(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    MatrixXd FR3XLSRobotData::computeJacobianActuated(const VectorXd& q_virtual,
                                                        const VectorXd& q_mobile,
                                                        const VectorXd& q_mani)
    {
        return drc::MobileManipulator::RobotData::computeJacobianActuated(q_virtual,q_mobile,q_mani,ee_name_);
    }

    MatrixXd FR3XLSRobotData::computeJacobianTimeVariationActuated(const VectorXd& q_virtual,
                                                                     const VectorXd& q_mobile,
                                                                     const VectorXd& q_mani,
                                                                     const VectorXd& qdot_virtual,
                                                                     const VectorXd& qdot_mobile,
                                                                     const VectorXd& qdot_mani)
    {
        return drc::MobileManipulator::RobotData::computeJacobianTimeVariationActuated(q_virtual,q_mobile,q_mani,qdot_virtual,qdot_mobile,qdot_mani,ee_name_);
    }

    drc::Manipulator::ManipulabilityResult FR3XLSRobotData::computeManipulability(const VectorXd& q_mani, 
                                                                                          const VectorXd& qdot_mani, 
                                                                                          const bool& with_grad, 
                                                                                          const bool& with_graddot)
    {
        return drc::MobileManipulator::RobotData::computeManipulability(q_mani,qdot_mani,with_grad,with_graddot,ee_name_);
    }

    Affine3d FR3XLSRobotData::getPose() const
    {
        return drc::MobileManipulator::RobotData::getPose(ee_name_);
    }

    MatrixXd FR3XLSRobotData::getJacobian()
    {
        return drc::MobileManipulator::RobotData::getJacobian(ee_name_);
    }

    MatrixXd FR3XLSRobotData::getJacobianTimeVariation()
    {
        return drc::MobileManipulator::RobotData::getJacobianTimeVariation(ee_name_);
    } 

    VectorXd FR3XLSRobotData::getVelocity()
    {
        return drc::MobileManipulator::RobotData::getVelocity(ee_name_);
    }   

    drc::Manipulator::ManipulabilityResult FR3XLSRobotData::getManipulability(const bool& with_grad, const bool& with_graddot)
    {
        return drc::MobileManipulator::RobotData::getManipulability(with_grad,with_graddot,ee_name_);
    }
    
    MatrixXd FR3XLSRobotData::getJacobianActuated()
    {
        return drc::MobileManipulator::RobotData::getJacobianActuated(ee_name_);
    }

    MatrixXd FR3XLSRobotData::getJacobianActuatedTimeVariation()
    {
        return drc::MobileManipulator::RobotData::getJacobianActuatedTimeVariation(ee_name_);
    }
} // namespace FR3XLS