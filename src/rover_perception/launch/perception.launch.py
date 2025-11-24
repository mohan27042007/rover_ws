from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_dir = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        ".."                     # go to rover_perception/
        )
    pkg_dir = os.path.realpath(pkg_dir)
    
  # when run from package root during local dev; otherwise set absolute paths
    boulder_model = os.path.join(pkg_dir, 'models', 'boulder_model.pt')
    crater_model = os.path.join(pkg_dir, 'models', 'crater_model.pt')

    return LaunchDescription([
        Node(
            package='rover_perception',
            executable='boulder_detector',
            name='boulder_detector',
            output='screen',
            parameters=[{'model_path': boulder_model, 'camera_topic': '/camera/image_raw'}]
        ),
        Node(
            package='rover_perception',
            executable='crater_detector',
            name='crater_detector',
            output='screen',
            parameters=[{'model_path': crater_model, 'camera_topic': '/camera/image_raw'}]
        ),
        Node(
            package='rover_perception',
            executable='perception_ensemble',
            name='perception_ensemble',
            output='screen'
        )
    ])
