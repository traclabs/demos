import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro

def generate_launch_description():

    launch_args =  [
      DeclareLaunchArgument(name='use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
      DeclareLaunchArgument(name='rviz', default_value='true')     
    ]
    
    ingenuity_path = get_package_share_directory("ingenuity_description")
    env_gz_resource = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", 
        os.pathsep.join(
            [
                os.environ.get("GZ_SIM_RESOURCE_PATH", default=""),
                ingenuity_path + "/models",
            ]
    ))
    
    # Paths
    ingenuity_path = get_package_share_directory('ingenuity_description')
    xacro_file = os.path.join(ingenuity_path, 'urdf', 'ingenuity_description.urdf')
    sdf_file_path = os.path.join(ingenuity_path, 'sdf', 'ingenuity_world.sdf')

    # Load URDF
    doc = xacro.process_file(xacro_file)
    urdf_content = doc.toxml()

    params = {'robot_description': urdf_content}
    print(urdf_content)

    gz_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
                launch_arguments=[('gz_args', [f'  -v 4 {sdf_file_path}'])]) #-r
    
    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    tf_world = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="world_base_tf_node",
    arguments=[
      "--frame-id", "world", "--child-frame-id", "ingenuity_world",
      "--x", "0.0", "--y", "0.0", "--z", "0.0", "--qx", "0.0", "--qy", "0.0", "--qz", "0.0", "--qw", "1.0"
    ],
    parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    output="screen",
    )

    tf_body = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="world_base_tf_node",
    arguments=[
      "--frame-id", "ingenuity/body", "--child-frame-id", "body",
      "--x", "0.0", "--y", "0.0", "--z", "0.0", "--qx", "0.0", "--qy", "0.0", "--qz", "0.0", "--qw", "1.0"
    ],
    parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    output="screen",
    )


    # Create a Node action to start RViz
    rviz_config_file = os.path.join(ingenuity_path, 'rviz', 'ingenuity_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration("rviz"))
    )
    
    # ROS 2 to Gazebo bridge for joint states and TF
    gz_ros2_bridge_yaml = os.path.join(get_package_share_directory('ingenuity_bringup'), 'config', 'gz_ros2_bridge.yaml')
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gz_ros2_bridge_yaml}],
        output='screen'
    )

    return LaunchDescription(
        launch_args + 
        [
        env_gz_resource,
        gz_launch,
        robot_state_publisher,
        bridge_node,
        rviz_node,
        tf_world,
        tf_body
        ]
    )    
