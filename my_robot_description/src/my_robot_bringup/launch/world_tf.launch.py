from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Xacro αρχείο του UGV
    ugv_xacro_file = PathJoinSubstitution([
        FindPackageShare("my_robot_description"),
        "urdf",
        "my_robot.urdf.xacro"
    ])

    robot_description_content = Command(["xacro ", ugv_xacro_file])
    robot_description = {"robot_description": robot_description_content}

    return LaunchDescription([
        # TF: world -> odom_ugv
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_world_to_odom_ugv",
            arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
            output="screen"
        ),

        # TF: odom_ugv -> base_footprint1 (UGV)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_odom_to_base_ugv",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
            output="screen"
        ),

        # UGV Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="ugv_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": True}]
        )
    ])
