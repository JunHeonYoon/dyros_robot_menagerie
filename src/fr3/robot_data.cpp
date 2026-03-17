#include "dyros_robot_menagerie/fr3/robot_data.h"

namespace FR3
{
    FR3RobotData::FR3RobotData(const double dt)
    : drc::Manipulator::RobotData(
        dt,
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3.urdf",
        ament_index_cpp::get_package_share_directory("dyros_robot_menagerie") + "/robot/fr3.srdf",
        ament_index_cpp::get_package_share_directory("mujoco_ros_sim")) 
    {
    }

} // namespace FR3
