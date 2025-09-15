import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    res = []

    # -----------------------
    # Robot model (URDF)
    # -----------------------
    urdf_path = os.path.join(
        get_package_share_directory("mycobot_description"),
        "urdf/mycobot_320_pi_2022/mycobot_320_pi_2022.urdf"
    )

    # read the file once, so we always pass contents not just path
    with open(urdf_path, 'r') as infp:
        urdf_content = infp.read()

    model_launch_arg = DeclareLaunchArgument(
        "model",
        default_value=urdf_path
    )
    res.append(model_launch_arg)

    # -----------------------
    # RViz config
    # -----------------------
    rvizconfig_launch_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("mycobot_320pi"),
            "config/mycobot_320_pi.rviz"
        )
    )
    res.append(rvizconfig_launch_arg)

    # -----------------------
    # Robot state publisher
    # -----------------------
    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': urdf_content}]
    )
    res.append(robot_state_publisher_node)

    # -----------------------
    # Initial pose publisher (one-shot)
    # -----------------------
    init_pose_pub = Node(
        package="nebula",
        executable="init_pose_publisher",
        name="init_pose_pub",
        output="screen"
    )
    res.append(init_pose_pub)

    # -----------------------
    # RViz2
    # -----------------------
    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=['-d', LaunchConfiguration("rvizconfig")],
    )
    res.append(rviz_node)

    # -----------------------
    # Floor marker node
    # -----------------------
    floor_marker_node = Node(
        package="nebula",
        executable="floor_marker",
        name="floor_marker",
        output="screen"
    )
    res.append(floor_marker_node)

    return LaunchDescription(res)
