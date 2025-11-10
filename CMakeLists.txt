cmake_minimum_required(VERSION 3.14)
project(dyros_robot_menagerie)
set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(mujoco_ros_sim         REQUIRED)
find_package(dyros_robot_controller REQUIRED)
find_package(rclcpp                 REQUIRED)
find_package(Eigen3                 REQUIRED)
find_package(std_msgs               REQUIRED)
find_package(geometry_msgs          REQUIRED)
find_package(sensor_msgs            REQUIRED)

include_directories(include)

# ===================== Husky =====================
add_library(husky_controller_plugins SHARED
    src/husky/controller.cpp
    src/husky/robot_data.cpp
)
ament_target_dependencies(husky_controller_plugins
    rclcpp 
    pluginlib 
    mujoco_ros_sim
)

target_link_libraries(husky_controller_plugins
    dyros_robot_controller::dyros_robot_controller
)

install(TARGETS husky_controller_plugins
        LIBRARY DESTINATION lib
)

install(PROGRAMS    ${PROJECT_NAME}/husky/ros2_qt_gui.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME      HuskyControllerQT
)

# ===================== XLS =====================
add_library(xls_controller_plugins SHARED
    src/xls/controller.cpp
    src/xls/robot_data.cpp
)
ament_target_dependencies(xls_controller_plugins
    rclcpp 
    pluginlib 
    mujoco_ros_sim
)

target_link_libraries(xls_controller_plugins
    dyros_robot_controller::dyros_robot_controller
)

install(TARGETS xls_controller_plugins
        LIBRARY DESTINATION lib
)

install(PROGRAMS    ${PROJECT_NAME}/xls/ros2_qt_gui.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME      XLSControllerQT
)


# ===================== PCV =====================
add_library(pcv_controller_plugins SHARED
    src/pcv/controller.cpp
    src/pcv/robot_data.cpp
)
ament_target_dependencies(pcv_controller_plugins
    rclcpp 
    pluginlib 
    mujoco_ros_sim
)

target_link_libraries(pcv_controller_plugins
    dyros_robot_controller::dyros_robot_controller
)

install(TARGETS pcv_controller_plugins
        LIBRARY DESTINATION lib
)

install(PROGRAMS    ${PROJECT_NAME}/pcv/ros2_qt_gui.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME      PCVControllerQT
)

# ===================== FR3 =====================
add_library(fr3_controller_plugins SHARED
    src/fr3/controller.cpp
    src/fr3/robot_data.cpp
)
ament_target_dependencies(fr3_controller_plugins
    rclcpp 
    pluginlib 
    mujoco_ros_sim
)

target_link_libraries(fr3_controller_plugins
    dyros_robot_controller::dyros_robot_controller
)

install(TARGETS fr3_controller_plugins
        LIBRARY DESTINATION lib
)

install(PROGRAMS    ${PROJECT_NAME}/fr3/ros2_qt_gui.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME      FR3ControllerQT
)

# ===================== UR5e =====================
add_library(ur5e_controller_plugins SHARED
    src/ur5e/controller.cpp
    src/ur5e/robot_data.cpp
)
ament_target_dependencies(ur5e_controller_plugins
    rclcpp 
    pluginlib 
    mujoco_ros_sim
)

target_link_libraries(ur5e_controller_plugins
    dyros_robot_controller::dyros_robot_controller
)

install(TARGETS ur5e_controller_plugins
        LIBRARY DESTINATION lib
)

install(PROGRAMS    ${PROJECT_NAME}/ur5e/ros2_qt_gui.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME      UR5eControllerQT
)
# ===================================================



install(DIRECTORY robot/    DESTINATION share/${PROJECT_NAME}/robot)
install(DIRECTORY launch/   DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/   DESTINATION share/${PROJECT_NAME}/config)
install(FILES plugins.xml   DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(mujoco_ros_sim plugins.xml)
            
ament_python_install_package(${PROJECT_NAME})
ament_package()