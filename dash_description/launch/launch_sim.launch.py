import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
import xacro
from nav2_common.launch import ReplaceString


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    robot_name = LaunchConfiguration('robot_name')
    world = LaunchConfiguration('world')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )
    
    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='false',
        description='Use ros2_control if true'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
      name='robot_name',
      default_value='dash',
      description='name of the robot')
    
    # declare_robot_name_cmd = DeclareLaunchArgument(
    #   name='world',
    #   default_value='dash',
    #   description='name of the robot')

    package_name='dash_description'

    share_dir = get_package_share_directory('dash_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'dash.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    # robot_urdf = robot_description_config.toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.sdf' #'industrial-warehouse.sdf' #'empty_world.sdf'
        )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load: tugbot_warehouse.sdf, industrial-warehouse.sdf, empty_world.world'
        )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')

    bridge_config = bridge_params
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_config}',
        ]
    )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', robot_name,
                                   '-z', '0.3'],
                        output='screen')

    # ros_gz_image_bridge = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",
    #     arguments=["/camera/image_raw"]
    # )
    # set some ignition environment variable
    gz_models_path = os.path.join(package_name, "meshes")
    gz_sim_system_plugin_path = '/opt/ros/humble/lib/'

    set_env_ign_resource_cmd = SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=gz_models_path,
        )
    
    set_env_ign_path_cmd = SetEnvironmentVariable(
            name="GZ_SIM_SYSTEM_PLUGIN_PATH",
            value=gz_sim_system_plugin_path,
        )
    
    rviz_config_file = os.path.join(get_package_share_directory(package_name),'config','display.rviz')

    # create needed nodes or launch files
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        declare_robot_name_cmd,
        set_env_ign_resource_cmd,
        set_env_ign_path_cmd,
        use_sim_time_arg,
        use_ros2_control_arg,
        robot_state_publisher_node,
        world_arg,
        gazebo,
        ros_gz_bridge,
        # ros_gz_image_bridge,
        spawn_entity,
        rviz_node
    ])