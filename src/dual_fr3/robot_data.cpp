#include "dyros_robot_menagerie/dual_fr3/robot_data.h"

namespace DualFR3
{

    DualFR3RobotData::DualFR3RobotData(const double dt)
    : drc::Manipulator::RobotData(
            dt,
            ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/dual_fr3.urdf",
            ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/dual_fr3.srdf",
            ament_index_cpp::get_package_share_directory("mujoco_ros_sim")) 
    {
    }
} // namespace DualFR3
