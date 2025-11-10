from launch import LaunchDescription
from launch import actions, substitutions
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    config_file = os.path.join(
        get_package_share_directory("dyros_robot_menagerie"),
        "config",
        "teleop_twist_joy.yaml"
    )
    
    declare_args = [
        actions.DeclareLaunchArgument('publish_stamped_twist',
            default_value=TextSubstitution(text='false')),
        actions.DeclareLaunchArgument('joy_dev',
            default_value=TextSubstitution(text='0')),
        actions.DeclareLaunchArgument('config_filepath',
            default_value=TextSubstitution(text=config_file)),
    ]
    
    # LaunchConfigurations
    publish_stamped_twist  = LaunchConfiguration('publish_stamped_twist')
    joy_dev                = LaunchConfiguration('joy_dev')
    config_filepath        = LaunchConfiguration('config_filepath')
    
    mujoco_ros_sim_node = Node(
        package='mujoco_ros_sim',
        executable='MujocoRosSim',
        name='mujoco_ros_sim_node',
        output='screen',
        parameters=[
            {'robot_name': "husky"},
            ## C++ controller
            {'controller_class': 'dyros_robot_menagerie/HuskyController'},
            ## Python controller
            # {'controller_class': 'dyros_robot_menagerie/HuskyControllerPy'},
        ],
    )
    
    relay_joint_states = Node(
        package='topic_tools',
        executable='relay',
        name='relay_joint_states_raw',
        arguments=['/joint_states', '/joint_states_raw'],
        output='screen'
    )
    
    
    urdf_path = os.path.join(get_package_share_directory('dyros_robot_menagerie'), 'robot', 'husky.urdf')
    srdf_path = os.path.join(get_package_share_directory('dyros_robot_menagerie'), 'robot', 'husky.srdf')
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
    with open(srdf_path, 'r') as infp:
        robot_description_semantic = infp.read()
        
    rviz_config_file = os.path.join(
        get_package_share_directory("dyros_robot_menagerie"),
        "launch", 
        "husky_rviz.rviz"
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
        executable='HuskyControllerQT',
        name='HuskyControllerQT',
        output='screen',
        emulate_tty=True,
    )
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }],
    )
    
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[
            config_filepath,
            {'publish_stamped_twist': publish_stamped_twist},
        ],
        remappings=[
            ('cmd_vel', '/husky_controller/cmd_vel'),
        ],
    )

    return LaunchDescription(
        declare_args +[
        mujoco_ros_sim_node,
        relay_joint_states,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,
        gui_node,
        joy_node,
        teleop_twist_joy_node,
        ]
    )