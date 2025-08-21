from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import xacro
import yaml
import os

def generate_launch_description():

    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path,
                                       'config'])

    # Additional command line arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)
    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    # Load robot_description and start robot_state_publisher
    # robot_description_content = ParameterValue(
    #     Command(['xacro ',
    #              PathJoinSubstitution([pkg_path,
    #                                    'urdf',
    #                                    'husky.urdf.xacro'])]),
    #     value_type=str)
    # robot_state_publisher_node = Node(package='robot_state_publisher',
    #                                   executable='robot_state_publisher',
    #                                   parameters=[{
    #                                       'robot_description': robot_description_content,
    #                                       'use_sim_time': use_sim_time
    #                                   }])
    # ld.add_action(robot_state_publisher_node)

    # Quadcopter - Load drone_description and start drone_state_publisher 
    # drone_description_content = ParameterValue(
    #     Command(['xacro ', 
    #              PathJoinSubstitution([FindPackageShare('sjtu_drone_description'),
    #                                    'urdf',
    #                                    'sjtu_drone.urdf.xacro'])]),
    #     value_type=str
    # )

    # drone_yaml_file_path = PathJoinSubstitution([
    #     FindPackageShare('sjtu_drone_description'),
    #     'config', 'drone.yaml'
    # ])   

    # drone_state_publisher_node = Node(package='drone_state_publisher',
    #                                   executable='drone_state_publisher',
    #                                   parameters=[{
    #                                       'robot_description': drone_description_content,
    #                                       'use_sim_time': use_sim_time
    #                                   }],
    #                                   remappings=[('/robot_description', '/drone_description')]
    #                                   )
    # ld.add_action(drone_state_publisher_node)

    drone_xacro_file_name = "sjtu_drone.urdf.xacro"
    drone_xacro_file = os.path.join(
        FindPackageShare('sjtu_drone_description').find('sjtu_drone_description'),
        "urdf", 
        drone_xacro_file_name
    )

    drone_yaml_file_path = os.path.join(
        FindPackageShare('41068_ignition_bringup').find('41068_ignition_bringup'),
        "config",
        "drone.yaml"
    )

    drone_description_config = xacro.process_file(str(drone_xacro_file), mappings={"params_path": str(drone_yaml_file_path)})
    drone_desc = drone_description_config.toxml()

    # get ns from yaml
    with open(drone_yaml_file_path, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        model_ns = yaml_dict["namespace"]
    print("namespace: ", model_ns)

    drone_state_publisher_node = Node(
        package='robot_state_publisher',
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=model_ns,
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": drone_desc,
            "frame_prefix": model_ns + "/"
        }],
        arguments=[drone_desc]
    )

    ld.add_action(drone_state_publisher_node)

    # Publish odom -> base_link transform **using robot_localization**
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_path,
                                          'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    # Start Gazebo to simulate the robot in the chosen world
    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo']
    )
    ld.add_action(world_launch_arg)
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                             'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg_path,
                                               'worlds',
                                               [LaunchConfiguration('world'), '.sdf']]),
                         ' -r']}.items()
    )
    ld.add_action(gazebo)

    # Spawn robot in Gazebo
    # robot_spawner = Node(
    #     package='ros_ign_gazebo',
    #     executable='create',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     arguments=['-topic', '/robot_description', '-z', '0.4']
    # )
    # ld.add_action(robot_spawner)

    # Spawn drone in Gazebo
    drone_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', 'simple_drone/robot_description', '-name', model_ns, '-z', '0.4']
    )
    ld.add_action(drone_spawner)

    # Bridge topics between gazebo and ROS2
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path,
                                                          'gazebo_bridge.yaml']),
                    'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    # rviz2 visualises data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path,
                                               '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # Nav2 enables mapping and waypoint following
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path,
                              'launch',
                              '41068_navigation.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    return ld
