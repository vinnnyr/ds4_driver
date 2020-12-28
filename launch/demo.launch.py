import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('ds4_driver'), 'launch')

    addr = LaunchConfiguration('addr')
    declare_addr = DeclareLaunchArgument(
        'addr',
        default_value='',
        description='The hardware address of the ds4')
    demo_node = Node(
        package='ds4_driver',
        executable='demo.py',
        name='demo',
        output='screen')
    include_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir,'ds4_driver.launch.py')),
        launch_arguments={'addr':addr}.items())

    # Generate blank launch description, populate it
    ld = LaunchDescription()

    ld.add_action(declare_addr)
    ld.add_action(demo_node)
    ld.add_action(include_driver)

    return ld
