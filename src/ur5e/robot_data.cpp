#include "dyros_robot_menagerie/ur5e/robot_data.h"

namespace UR5e
{
    UR5eRobotData::UR5eRobotData()
    : drc::Manipulator::RobotData(
            ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/ur5e.urdf",
            ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/ur5e.srdf",
            ament_index_cpp::get_package_share_directory("mujoco_ros_sim")) 
    {
        ee_name_ = "ee_link";
    }

    Affine3d UR5eRobotData::computePose(const VectorXd& q)
    {
        return drc::Manipulator::RobotData::computePose(q, ee_name_);
    }

    MatrixXd UR5eRobotData::computeJacobian(const VectorXd& q)
    {
        return drc::Manipulator::RobotData::computeJacobian(q, ee_name_);
    }

    MatrixXd UR5eRobotData::computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot)
    {
        return drc::Manipulator::RobotData::computeJacobianTimeVariation(q, qdot, ee_name_);
    }

    VectorXd UR5eRobotData::computeVelocity(const VectorXd& q, const VectorXd& qdot)
    {
        return drc::Manipulator::RobotData::computeVelocity(q, qdot, ee_name_);
    }

    drc::Manipulator::ManipulabilityResult UR5eRobotData::computeManipulability(const VectorXd& q, const VectorXd& qdot, const bool& with_grad, const bool& with_graddot)
    {
        return drc::Manipulator::RobotData::computeManipulability(q, qdot, with_grad, with_graddot, ee_name_);
    }
    
    Affine3d UR5eRobotData::getPose() const
    {
        return drc::Manipulator::RobotData::getPose(ee_name_);
    }

    MatrixXd UR5eRobotData::getJacobian()
    {
        return drc::Manipulator::RobotData::getJacobian(ee_name_);
    }

    MatrixXd UR5eRobotData::getJacobianTimeVariation()
    {
        return drc::Manipulator::RobotData::getJacobianTimeVariation(ee_name_);
    } 

    VectorXd UR5eRobotData::getVelocity()
    {
        return drc::Manipulator::RobotData::getVelocity(ee_name_);
    }

    drc::Manipulator::ManipulabilityResult UR5eRobotData::getManipulability(const bool& with_grad, const bool& with_graddot)
    {
        return drc::Manipulator::RobotData::getManipulability(with_grad, with_graddot, ee_name_);
    }
} // namespace UR5e