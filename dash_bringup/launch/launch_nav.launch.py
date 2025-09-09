import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
import xacro
from nav2_common.launch import ReplaceString


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    # use_ros2_control = LaunchConfiguration('use_ros2_control')
    map_name = LaunchConfiguration('map_name')
    
    

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='turtlebot_arena.yaml',
        description='Name of the map to load: Available maps are turtlebot_arena.yaml, industrial-warehouse.yaml, tugbot_warehouse.yaml'
    )

    map_path = PathJoinSubstitution([
        get_package_share_directory('dash_nav2'),
        "maps",
        map_name,
    ])

    # package_name='dash_description'
    # nav2_pkg_name='dash_nav2'

    # share_dir = get_package_share_directory('dash_description')

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    nav = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('dash_nav2'), 'launch', 'dash_navbringup.launch.py')]),
                    launch_arguments={'slam': 'False', 'use_sim_time': use_sim_time, 'map': map_path}.items()
             )

    # Launch them all!
    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        nav
    ])