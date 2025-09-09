from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.actions import AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('dash_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'dash.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    default_world = os.path.join(
        get_package_share_directory('dash_description'),
        'worlds',
        'empty_world.world'
        )    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )



    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    #gazebo_server = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([
    #        PathJoinSubstitution([
    #            FindPackageShare('ros_gz_sim'),
    #            'launch',
    #            'gz_sim.launch.py'
    #        ])
    #    ]),
    #    #launch_arguments={
    #    #    'on_exit_shutdown': 'true'
    #    #}.items()
    #    launch_arguments={'gz_args': ['-r -s -v4 '], 'on_exit_shutdown': 'true'}.items()
    #)
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        #launch_arguments={
        #    'on_exit_shutdown': 'true'
        #}.items()
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    # gazebo = ExecuteProcess(
    #     cmd=['gz', 'sim', '-r', '-s', '-v4', world],
    #     output='screen',
    #     name='gazebo'
    # )

    #gazebo_client = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([
    #        PathJoinSubstitution([
    #            FindPackageShare('ros_gz_sim'),
    #            'launch',
    #            'gz_sim.launch.py'
    #        ])
    #    ]),
    #    launch_arguments={'gz_args': '-g -v4 '}.items()
    #)
    
    
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('dash_description'),
                     'meshes'))
   
    
    #urdf_spawn_node = Node(
    #    package='gazebo_ros',
    #    executable='spawn_entity.py',
    #    arguments=[
    #        '-entity', 'dash',
    #        '-topic', 'robot_description'
    #    ],
    #    output='screen'
    #)
    
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'dash1',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    bridge_params = os.path.join(get_package_share_directory('dash_description'),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    return LaunchDescription([
        set_env_vars_resources,
        world_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo,
        #gazebo_server,
        #gazebo_client,
        #urdf_spawn_node,
        start_gazebo_ros_spawner_cmd,
        # ros_gz_bridge,
    ])
