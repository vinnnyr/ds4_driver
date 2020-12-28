import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # get commonly needed directories
    share_dir = get_package_share_directory('ds4_driver')
    launch_dir = os.path.join(share_dir, 'launch')

    ## getting dof
    dof = LaunchConfiguration('dof')
    dof_numb = dof.perform(context)

    addr = LaunchConfiguration('addr')

    dof = LaunchConfiguration('dof')


    stamped = LaunchConfiguration('stamped')
    param_config = os.path.join(
        share_dir,
        'config',
        'twist_{}dof.yaml'.format(dof_numb))
    # read parameters
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['ds4_twist']['ros__parameters']

    print(params)
    
    # overwrite config file with declared parameters
    params['addr'] = addr
    params['stamped'] = stamped


    twist_node = Node(
        package='ds4_driver',
        executable='ds4_twist_node.py',
        name='ds4_twist',
        output='screen',
        parameters=[params])

    include_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir,'ds4_driver.launch.py')),
        launch_arguments={'addr':addr}.items())

    return [twist_node, include_driver]
    

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
        'addr',
        default_value='',
        description='The hardware address of the ds4'),
        DeclareLaunchArgument(
        'dof',
        default_value='6',
        description='The degrees of freedom desired in the Twist msg'),
        DeclareLaunchArgument(
        'stamped',
        default_value='false',
        description='Publish TwistStamped instead of Twist?'),
        OpaqueFunction(function = launch_setup)
    ])