from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node, PushRosNamespace

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    arg = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(arg)

    package_path = get_package_share_path('depth_control')
    controller_params_file_path = str(package_path / 'config/controller_params.yaml')
    calculator_params_file_path = str(package_path / 'config/calculator_params.yaml')
    setpoint_params_file_path = str(package_path / 'config/setpoint_params.yaml')
    # expose the parameter to the laumch command line
    controller_params_file_arg = DeclareLaunchArgument('controller_config_file',
                                           default_value=controller_params_file_path)
    calculator_params_file_arg = DeclareLaunchArgument('calculator_config_file',
                                            default_value=calculator_params_file_path)
    setpoint_params_file_arg = DeclareLaunchArgument('setpoint_config_file',
                                            default_value=setpoint_params_file_path)
    launch_description.add_action(controller_params_file_arg)
    launch_description.add_action(calculator_params_file_arg)
    launch_description.add_action(setpoint_params_file_arg)

    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        Node(executable='depth_calculator.py', package='depth_control', parameters=[LaunchConfiguration('calculator_config_file')]),
        Node(executable='depth_controller.py', package='depth_control', parameters=[LaunchConfiguration('controller_config_file')]),
        Node(executable='depth_setpoint.py', package='depth_control', parameters=[LaunchConfiguration('setpoint_config_file')]),
    ])
    launch_description.add_action(group)
    return launch_description
