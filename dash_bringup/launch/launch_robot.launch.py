import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import xacro
from nav2_common.launch import ReplaceString


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    robot_name = LaunchConfiguration('robot_name')
    world = LaunchConfiguration('world')
    use_joystick = LaunchConfiguration('use_joystick')
    use_lidar = LaunchConfiguration('lidar')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    )
    
    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control if true'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
      name='robot_name',
      default_value='dash',
      description='name of the robot')

    declare_use_joystick_cmd = DeclareLaunchArgument(
      name='use_joystick',
      default_value='true',
      description='Whether to use joystick control'
    )

    declare_lidar_cmd = DeclareLaunchArgument(
      name='lidar',
      default_value='true',
      description='run lidar')

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
    robot_controllers = os.path.join(
        get_package_share_directory('dash_description'),
        'config',
        'dash_diff_drive_controller.yaml'
    )


    controller_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[params, robot_controllers],
        output='both',
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        remappings=[
            # ("/cmd_vel", "/diff_cont/cmd_vel_unstamped"),
            ("/diff_cont/cmd_vel_unstamped", "/cmd_vel"),
        ],
        output="screen",
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    start_dash_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    #Lidar

    laser_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("rplidar_ros").find("rplidar_ros"),
                "launch",
                "rplidar_a1_launch.py"
            )
        ),
        launch_arguments={
            "serial_port": "/dev/ttyUSB0",
            "frame_id": "laser_frame",
            "inverted": "false",
            "angle_compensate": "true",
            "scan_mode": "Standard"
        }.items(),
        condition=IfCondition(use_lidar)
    )
    

    #twix
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    #Joystick

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("dash_bringup"),
                "launch",
                "joystick.launch.py"
            )
        ),
        condition=IfCondition(use_joystick)
    )

    #odom

    laser_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                os.path.join(
                get_package_share_directory("rf2o_laser_odometry"),
                "launch",
                "rf2o_laser_odometry.launch.py"
            )
        ),
        condition=IfCondition(use_lidar)   
    )

    # Add in your launch_robot.launch.py
    wheel = Node(
    package='dash_bringup',
    executable='virtual_wheel_states.py',
    name='virtual_wheel_states',
    parameters=[{
        'wheel_radius': 0.033,
        'wheel_separation': 0.287,
        'use_odom': True
    }]
    )
    
    return LaunchDescription([
        declare_use_joystick_cmd,
        declare_robot_name_cmd,
        use_sim_time_arg,
        use_ros2_control_arg,
        declare_lidar_cmd,
        controller_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        start_dash_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner,
        laser_interface,
        twist_mux,
        joystick,
        laser_odom,
        wheel
    ])