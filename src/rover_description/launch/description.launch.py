from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.xacro')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command([
                        'xacro ',
                        urdf_path
                    ]),
                    value_type=str
                )
            }]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        )
    ])
