from http.server import executable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnExecutionComplete
import os
from os import environ

from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():

    CANADARM_DEMOS_PATH = get_package_share_directory('canadarm')

    urdf_model_path = os.path.join(get_package_share_directory('simulation'), 'models', 'canadarm', 'urdf', 'SSRMS_Canadarm2.urdf.xacro')
    robot_xacro = xacro.process_file(
      urdf_model_path,
      mappings={'xyz' : '1.0 0.0 1.5', 'rpy': '3.1416 0.0 0.0'}
    )
    robot_description = {'robot_description': robot_xacro.toxml()}
    srdf_path = os.path.join(
      get_package_share_directory('canadarm_moveit_config'),
      'config',
      'SSRMS_Canadarm2.srdf'
    )
    robot_description_semantic = None
    with open(srdf_path, "r") as file:
      robot_description_semantic = file.read()

    gz_world_launch_arg = DeclareLaunchArgument(
        "gz_world",
        description="Name of world file to load from canadarm/worlds/", 
        default_value=TextSubstitution(text="lunar.world")
    )

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_RESOURCE_PATH':
           ':'.join([environ.get('IGN_GAZEBO_RESOURCE_PATH', default=''), CANADARM_DEMOS_PATH])}

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/image_raw', '/image_raw'],
        output='screen')

    run_move_arm = Node(
        package="canadarm",
        executable="move_arm",
        output='screen'
    )

    start_world = ExecuteProcess(
        cmd=['ign gazebo',
             PathJoinSubstitution([CANADARM_DEMOS_PATH, 'worlds', LaunchConfiguration('gz_world')]),
             '-r'],
        output='screen',
        additional_env=env,
        shell=True
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    parameters = {
      'robot_description': robot_xacro.toprettyxml(indent="    "),
      'robot_description_semantic': robot_description_semantic,
      'group': "canadarm",
      'reference_frame': 'Base_SSRMS',
    }

    test_search = Node(package='sandbox',
      executable='demo_canadarm_search',
      output='screen',
      parameters=[parameters],
      arguments=['--ros-args', '--log-level', ['CraftsmanSearch:=','debug', 'CollisionDetection:=','debug']]
    )

    spawn = Node(
        package='ros_ign_gazebo', executable='create',
        arguments=[
            '-name', 'canadarm',
            '-topic', robot_description,
        ],
        output='screen'
    )

    # Control
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_canadarm_joint_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'canadarm_joint_trajectory_controller'],
        output='screen'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", "src/demos/canadarm/config/demo.rviz"]
    )

    return LaunchDescription(
        [
            gz_world_launch_arg,
            start_world,
            robot_state_publisher,
            spawn,
            image_bridge,
            rviz_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_canadarm_joint_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_canadarm_joint_controller,
                    on_exit=[test_search],
                )
            ),
        ]
    )
