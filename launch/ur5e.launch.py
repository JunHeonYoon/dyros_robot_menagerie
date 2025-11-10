from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mujoco_ros_sim_node = Node(
        package='mujoco_ros_sim',
        executable='MujocoRosSim',
        name='mujoco_ros_sim_node',
        output='screen',
        parameters=[
            {'robot_name': 'universal_robots_ur5e'},
            ## C++ controller
            {'controller_class': 'dyros_robot_menagerie/UR5eController'},
            ## Python controller
            # {'controller_class': 'dyros_robot_menagerie/UR5eControllerPy'},
        ],
        # prefix='gdb -ex run --args'
    )


    urdf_path = os.path.join(get_package_share_directory('dyros_robot_menagerie'), 'robot', 'ur5e.urdf')
    srdf_path = os.path.join(get_package_share_directory('dyros_robot_menagerie'), 'robot', 'ur5e.srdf')
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
    with open(srdf_path, 'r') as infp:
        robot_description_semantic = infp.read()
        
    rviz_config_file = os.path.join(
        get_package_share_directory("dyros_robot_menagerie"),
        "launch", 
        "ur5e_rviz.rviz"
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
            'robot_description_semantic': robot_description_semantic
        }],
    )
    
    gui_node = Node(
        package='dyros_robot_menagerie',
        executable='UR5eControllerQT',
        name='UR5eControllerQT',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        mujoco_ros_sim_node,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,
        gui_node,
    ])