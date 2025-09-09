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
    # use_ros2_control = LaunchConfiguration('use_ros2_control')
    # robot_name = LaunchConfiguration('robot_name')
    
    

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )

    # package_name='dash_description'
    # nav2_pkg_name='dash_nav2'

    # share_dir = get_package_share_directory('dash_description')

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    mapping = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('dash_nav2'), 'launch', 'dash_bringup.launch.py')]),
                    launch_arguments={'slam': 'True', 'use_sim_time': use_sim_time}.items()
             )

    # Launch them all!
    return LaunchDescription([
        use_sim_time_arg,
        mapping
    ])