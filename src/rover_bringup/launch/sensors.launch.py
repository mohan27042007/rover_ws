from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    robot_xacro = PathJoinSubstitution([
        FindPackageShare("rover_description"),
        "urdf",
        "robot.xacro"
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            robot_xacro
        ]),
        value_type=str
    )


    return LaunchDescription([

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": True
            }],
            output="screen"
        ),

        # Parameter bridge (manual topic definitions)
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                "/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
                "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V"
            ],
            output="screen"
        )
    ])

