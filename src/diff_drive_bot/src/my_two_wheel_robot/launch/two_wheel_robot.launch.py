import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('my_two_wheel_robot')
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_two_wheel_robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'urdf.rviz')
    controller_params_file = os.path.join(pkg_share, 'config', 'diff_drive_controllers.yaml')

    robot_description_substitution = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(robot_description_substitution, value_type=str)
    robot_description_param = {'robot_description': robot_description}

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            robot_description_param,
            controller_params_file
        ]
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        ros2_control_node,
        joint_state_spawner,
        diff_drive_spawner,
        rviz_node,
    ])