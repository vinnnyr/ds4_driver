import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    # get commonly needed directories
    share_dir = get_package_share_directory('ds4_driver')
    launch_dir = os.path.join(share_dir, 'launch')

    # get config file
    param_config = os.path.join(
        share_dir,
        'config',
        'params.yaml'
    )
    
    # read parameters
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['ds4_driver']['ros__parameters']

    print(params)

    # declare launch configurations
    addr = LaunchConfiguration('addr')
    use_standard_msgs = LaunchConfiguration('use_standard_msgs')
    autorepeat_rate = LaunchConfiguration('autorepeat_rate')

    # declare actions
    declare_addr = DeclareLaunchArgument(
        'addr',
        default_value='',
        description='The hardware address of the ds4')
    declare_use_standard_msgs = DeclareLaunchArgument(
        'use_standard_msgs',
        default_value='false',
        description='Determine if standard joy and joy feedback messages should be published')
    declare_autorepeat_rate = DeclareLaunchArgument(
        'autorepeat_rate',
        default_value="0",
        description='The autorepeat rate of the messages if using standard messages')

    # overwrite config file with declared parameters
    params['addr'] = addr
    params['use_standard_msgs'] = use_standard_msgs
    params['autorepeate_rate'] = autorepeat_rate

    driver_node = Node(
        package='ds4_driver',
        executable='ds4_driver_node.py',
        name='ds4_driver',
        output='screen',
        parameters=[params])

    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ds4_to_imu',
        arguments=['0', '0.05', '-0.01', '-1.5707',
        '0', '1.5707', 'ds4', 'ds4_imu'],
        output='screen')

    # generate launch description, populate
    ld = LaunchDescription()

    ld.add_action(declare_addr)
    ld.add_action(declare_use_standard_msgs)
    ld.add_action(declare_autorepeat_rate)
    ld.add_action(driver_node)
    ld.add_action(static_transform_node)

    return ld
