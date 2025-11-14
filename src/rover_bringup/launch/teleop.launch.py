from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_key',
        output='screen',
        prefix='xterm -e',
        parameters=[]
    )

    return LaunchDescription([teleop])
