import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler

from launch_ros.actions import Node
import xacro
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'romeona_description'
    file_subpath = 'description/example_robot.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("romeona_description"),
            "config",
            "myrobot_controllers.yaml",
        ]
    )

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    rviz_file_name = 'description_rviz.rviz'
    rviz_file_path = os.path.join(
        get_package_share_directory('romeona_description'),
        'config',
        rviz_file_name
    )
    rviz_Node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_file_path],
        output='screen')


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'robot'],
                    output='screen')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description_raw}, robot_controllers],

        output="both",
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
    )

    generator = Node(
        package="romeona_control",
        executable="generator.py",
    )
    proximity_detector = Node(
        package="romeona_control",
        executable="proximity_detector.py",
    )
    scheduler = Node(
        package="romeona_control",
        executable="scheduler.py",
        
    )
    tracker = Node(
        package="romeona_control",
        executable="tracker.py",
        
    )
    marker = Node(
        package="romeona_control",
        executable="marker.py",
        
    )

    # Run the node
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        control_node,
        rviz_Node,
        TimerAction(actions = [tracker], period=5.0),
        TimerAction(actions = [generator], period=6.0),
        TimerAction(actions = [proximity_detector], period=7.0),
        TimerAction(actions = [marker], period=8.0),
        TimerAction(actions = [scheduler], period=9.0),
        ])
        


