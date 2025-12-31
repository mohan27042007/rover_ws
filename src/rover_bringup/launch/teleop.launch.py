from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    teleop = Node(
        package='rover_bringup',
        executable='custom_teleop.py',
        name='teleop_key',
        output='screen',
        prefix='xterm -e',
        parameters=[]
    )

    return LaunchDescription([teleop])
